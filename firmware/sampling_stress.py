"""Instruction-cost stress for the real FLASH app's first feedback windows.

This is an adverse CPU-load model, not cycle-accurate silicon timing evidence.
The existing peripheral timing and independent IRQ handlers remain active.
"""
import hashlib
import json
from pathlib import Path
import sys
import application_thumb as app
from unicorn.arm_const import UC_ARM_REG_PC,UC_ARM_REG_LR,UC_ARM_REG_IPSR


class CostedCore(app.ApplicationCore):
    def __init__(self,data,symbols,cost):
        super().__init__(data,symbols,'normal')
        self.cost=cost
        self.cycle_remainder=0
        self.first_error=None

    def write(self,cpu,access,address,size,value,user):
        if address==self.symbols['motor']+249 and value and self.first_error is None:
            motor=self.symbols['motor']
            self.first_error={'pc':hex(cpu.reg_read(UC_ARM_REG_PC)),
                'lr':hex(cpu.reg_read(UC_ARM_REG_LR)),'virtual_us':self.virtual_us,
                'phase':int.from_bytes(cpu.mem_read(motor+248,1),'little'),
                'arm_cycles':self.get(motor+260),'clamp_cycles':self.get(motor+264),
                'next_sample':self.get(motor+268),'timer_count':self.get(app.service.TIM+0x24),
                'ipsr':cpu.reg_read(UC_ARM_REG_IPSR),
                'calibration_state_word':hex(self.get(self.symbols['valve']+332)),
                'adc_launches':len(self.adc_launches),'adc_events':len(self.adc_events)}
        super().write(cpu,access,address,size,value,user)

    def code(self,cpu,address,size,user):
        super().code(cpu,address,size,user)
        if self.code_base<=address<self.code_base+len(self.data):
            elapsed,self.cycle_remainder=divmod(self.cycle_remainder+self.cost,48)
            if elapsed:self.advance_us(elapsed)


def run(data,symbols,cost):
    model=CostedCore(data,symbols,cost)
    model.boot()
    app.until(model,lambda:model.virtual_us>=250000)
    model.rx.extend(app.base.encode(app.base.Frame(0x71,2,bytes.fromhex('07090001'))))
    app.until(model,lambda:model.motor_report()[5]>=3 or model.motor_report()[3]!=0)
    observed=model.motor_report()
    model.rx.extend(app.base.encode(app.base.Frame(0x72,2,bytes.fromhex('07030000'))))
    app.until(model,lambda:model.motor_report()[2] in (7,8))
    final=model.motor_report()
    assert model.off() and final[4]==1 and final[10:13]==(0,0,0)
    return {'instruction_cost_cycles':cost,'first_windows_samples':observed[5],
            'driver_error':observed[3],'final_driver_error':final[3],
            'stopped':True,'virtual_us':model.virtual_us,'instructions':model.instructions,
            'first_error':model.first_error,
            'hardware_tested':False,'cycle_accurate':False}


def main():
    directory=Path(sys.argv[1])
    require_clean=len(sys.argv)==3 and sys.argv[2]=='--require-clean'
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    results=[]
    for cost in (2,3,4):
        result=run(data,manifest['symbols'],cost)
        results.append(result)
        print(json.dumps(result),flush=True)
        if require_clean:
            assert result['first_windows_samples']>=3 and not result['driver_error'] and not result['final_driver_error'],result
    (directory/'sampling-stress-results.json').write_text(json.dumps({
        'complete':True,'bin_sha256':manifest['bin_sha256'],'cases':results,
        'all_clean':all(not r['driver_error'] and r['first_windows_samples']>=3 for r in results),
        'cycle_accurate':False,'hardware_tested':False,
        'model_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},indent=2)+'\n')


if __name__=='__main__':
    main()
