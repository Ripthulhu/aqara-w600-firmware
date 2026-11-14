"""Real linked app with a one-byte UART receive register and 115200-baud wire.

Instruction costs are adverse loads, not cycle-accurate silicon evidence.
Unlike the older queue model, bytes arriving before RDR is drained overrun.
"""
import hashlib
import json
from pathlib import Path
import sys

from sampling_stress import CostedCore
import application_thumb as app
from unicorn.arm_const import UC_ARM_REG_LR


class WireCore(CostedCore):
    def __init__(self,data,symbols,cost):
        self.wire=[];self.overruns=0;self.arrived=0;self.first_overrun=None
        self.mask_since=None;self.mask_pc=None
        self.rx_at=None;self.last_masked_span=None;self.max_receive_age=0
        super().__init__(data,symbols,cost)

    def schedule(self,frame,at):
        self.wire.extend((at+i*87,byte) for i,byte in enumerate(app.base.encode(frame)))
        self.wire.sort()

    def advance_us(self,elapsed):
        super().advance_us(elapsed)
        while self.wire and self.wire[0][0]<=self.virtual_us:
            _,byte=self.wire.pop(0);self.arrived+=1
            if self.rx:
                if self.first_overrun is None:
                    self.first_overrun=dict(virtual_us=self.virtual_us,elapsed_us=elapsed,
                        masked=self.cpu.reg_read(app.base.UC_ARM_REG_PRIMASK),
                        in_handler=self.in_handler,reason=self.reason,
                        pc=hex(self.cpu.reg_read(app.base.UC_ARM_REG_PC)),
                        lr=hex(self.cpu.reg_read(UC_ARM_REG_LR)),
                        buffered_at_us=self.rx_at,buffered_age_us=None if self.rx_at is None else self.virtual_us-self.rx_at,
                        last_masked_span=self.last_masked_span,
                        masked_since_us=self.mask_since,masked_start_pc=self.mask_pc,
                        masked_for_us=None if self.mask_since is None else self.virtual_us-self.mask_since)
                self.uart_error|=8;self.overruns+=1
            else:self.rx.append(byte);self.rx_at=self.virtual_us

    def read(self,cpu,access,address,size,value,user):
        if address==app.base.UART+0x24 and self.rx_at is not None:
            self.max_receive_age=max(self.max_receive_age,self.virtual_us-self.rx_at)
            self.rx_at=None
        super().read(cpu,access,address,size,value,user)

    def step(self,elapsed=125):
        # Yield idle fast-forward at each arrival so enabled IRQs can run.
        if self.reason=='foreground_boundary':
            if self.rx or self.pending_tick:elapsed=0
            elif self.wire:elapsed=min(elapsed,max(0,self.wire[0][0]-self.virtual_us))
        super().step(elapsed)

    def code(self,cpu,address,size,user):
        masked=cpu.reg_read(app.base.UC_ARM_REG_PRIMASK)
        if masked and self.mask_since is None:
            self.mask_since=self.virtual_us;self.mask_pc=hex(address)
        elif not masked and self.mask_since is not None:
            self.last_masked_span=dict(start_us=self.mask_since,end_us=self.virtual_us,
                duration_us=self.virtual_us-self.mask_since,start_pc=self.mask_pc)
            self.mask_since=None;self.mask_pc=None
        super().code(cpu,address,size,user)


def run(data,manifest,scenario):
    m=WireCore(data,manifest['symbols'],3);m.boot()
    app.until(m,lambda:m.virtual_us>=(250000 if scenario=='calibration' else 990000))
    at=m.virtual_us+1000
    first=app.base.Frame(0x71,2,bytes.fromhex('07090001')) if scenario=='calibration' else app.base.Frame(0x71,1,bytes.fromhex('07000706'))
    m.schedule(first,at)
    for i in range(8):
        m.schedule(app.base.Frame(0x72+i,1,bytes.fromhex('07000706')),at+10000+i*20000)
    app.until(m,lambda:m.reason=='fatal' or m.virtual_us>=at+200000)
    if scenario=='calibration' and m.reason!='fatal':
        m.schedule(app.base.Frame(0x7f,2,bytes.fromhex('07030000')),m.virtual_us+1000)
        app.until(m,lambda:m.reason=='fatal' or m.motor_report()[2] in (7,8))
    clean=not m.overruns and m.report()[2:4]==(2,0) and m.motor_report()[3]==0
    if clean:
        assert m.off() and m.motor_report()[10:13]==(0,0,0)
        if scenario=='sensor':assert m.report()[23]>=1 and m.report()[22]==1
    result=dict(bin_sha256=manifest['bin_sha256'],one_byte_receive_register=True,
        byte_interval_us=87,instruction_cost_cycles=3,overruns=m.overruns,
        bytes_arrived=m.arrived,app_report=m.report(),motor_report=m.motor_report(),
        first_overrun=m.first_overrun,idle_fast_forward_yields_at_arrivals=True,
        nested_irq_count=m.nested_irq_count,max_irq_depth=m.max_irq_depth,
        uart_priority=m.irq_priority(43),systick_priority=m.irq_priority(15),
        max_receive_age_us=m.max_receive_age,
        virtual_us=m.virtual_us,instructions=m.instructions,stopped=m.off(),
        hardware_tested=False,cycle_accurate=False,
        model_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    result.update(scenario=scenario,clean=clean)
    return result

def main():
    directory=Path(sys.argv[1]);manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    require_clean=sys.argv[2:]==['--require-clean']
    assert sys.argv[2:] in ([],['--require-clean'])
    results={}
    for scenario in ('calibration','sensor'):
        results[scenario]=run(data,manifest,scenario)
        print(json.dumps(results[scenario]),flush=True)
        if require_clean:assert results[scenario]['clean'],results[scenario]
    (directory/'uart-wire-results.json').write_text(json.dumps(dict(
        complete=True,bin_sha256=manifest['bin_sha256'],cases=results,
        all_clean=all(r['clean'] for r in results.values()),hardware_tested=False,
        cycle_accurate=False),indent=2)+'\n')

if __name__=='__main__':main()
