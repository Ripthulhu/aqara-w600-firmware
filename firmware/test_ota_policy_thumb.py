"""Run each real ARM image's OTA policy against real subsequent build images.

Calls the compiled policy and compiled flash reader; does not emulate radio,
bootloader installation, physical flash programming or motor hardware.
"""
import hashlib
import json
from pathlib import Path
import struct
import sys
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3,UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC
from ota_image import validate

def check(running,candidate,revision,busy=False):
    data,manifest=running
    m=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
    m.mem_map(0x08000000,0x20000);m.mem_map(0x20000000,0x8000);m.mem_map(0x40022000,0x1000)
    m.mem_write(0x08002000,data);m.mem_write(0x08010800,candidate)
    m.mem_write(0x40022010,struct.pack('<I',0x10000 if busy else 0))
    sp=0x20006ff0
    m.mem_write(sp,struct.pack('<3I',revision,manifest['controller_revision'],0x20001000))
    for register,value in ((UC_ARM_REG_R0,manifest['symbols']['ota_stage_read']|1),
                           (UC_ARM_REG_R1,0),(UC_ARM_REG_R2,0x08010800),(UC_ARM_REG_R3,len(candidate)),
                           (UC_ARM_REG_SP,sp),(UC_ARM_REG_LR,0x08000001)):
        m.reg_write(register,value)
    m.emu_start(manifest['symbols']['w600_ota_authorize_image']|1,0x08000000,count=10000000)
    assert m.reg_read(UC_ARM_REG_PC)==0x08000000,'Compiled policy did not return within the instruction bound'
    return m.reg_read(UC_ARM_REG_R0)==1

def load(path):
    path=Path(path);m=json.loads((path/'manifest.json').read_bytes());data=(path/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==m['bin_sha256'] and m['ota_commit_authorization_enabled']
    validate(data,m['controller_revision'])
    return data,m

if __name__=='__main__':
    paths=list(map(Path,sys.argv[1:]));assert len(paths)==3
    builds=list(map(load,paths));cases=[]
    for running,next_build in zip(builds,builds[1:]):
        image,metadata=next_build;revision=metadata['controller_revision']
        assert check(running,image,revision)
        assert not check(next_build,image,revision)
        assert not check(running,image,revision,busy=True)
        bad=bytearray(image);bad[300]^=1
        assert not check(running,bytes(bad),revision)
        assert not check(running,image,revision+1)
        cases.append(dict(source=running[1]['controller_revision'],target=revision,
                          source_sha256=running[1]['bin_sha256'],target_sha256=metadata['bin_sha256'],
                          accepted=True,replay_corruption_busy_and_revision_mismatch_rejected=True))
    result=dict(complete=True,actual_arm_policy_and_reader=True,cases=cases,hardware_tested=False)
    (paths[0]/'ota-policy-thumb-results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
