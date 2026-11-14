"""Prepare a valve-2 development-image diff. This never accesses a device."""
import argparse
import hashlib
import json
from pathlib import Path
import struct

BASELINE_SHA="6c3be03f8935e25336fa542e0db7490aea1ce6b4e2dd9928c45e85998b394cbb"
INSTALLED_CONTROL_V2_SHA="24c164ce6c40eea18841b386b7688267541bcbbddfd98823b6084a0d468094d4"
INSTALLED_CONTROL_V8_SHA="3720a994f6455d5f82b4976f428fdcf809bbee4b63f6b35aa7b8c2a64ba9103b"
INSTALLED_CONTROL_V11_SHA="6238b97e8496240bcf1ac90ce7c15976cf1f3cd94cf70aa5d1ab9e9a75a665e3"
INSTALLED_CONTROL_V12_SHA="a7164a47d87a874bfacc4ea7ac14240bac454cc89edf92b541243eebf492551a"
INSTALLED_CONTROL_V13_SHA="928b1d8d16a896c8958e257200bb1a07a109a1afdbd1fea151aa8ea7e0747c6b"
INSTALLED_CONTROL_V14_SHA="6c3db83ac2afad53dd74d0adec8781d11f1becd02997fa074578cb09dfa1d30b"
INSTALLED_CONTROL_V15_SHA="fe24219b2094e8ee695204cc1dedd81ee1fd16bc3664c38d184188bca9123e33"
INSTALLED_CONTROL_R6_SHA="b0353f9acf810e22aef0f4f72d676c78e5cafcfcdd7cbcb9452cadd67e355361"
INSTALLED_CONTROL_R11_SHA="1ccffd126f24515b082fdda75cfc49a88b33ff563184c9e4340bc8e7f8d7a096"
BASE=0x08000000
START=0x2000
END=0xee00
PAGE=512


def sha(data):
    return hashlib.sha256(data).hexdigest()


def image_plan(original,image,manifest,*,accepted_source_sha=None):
    accepted=(BASELINE_SHA,INSTALLED_CONTROL_V2_SHA,INSTALLED_CONTROL_V8_SHA,INSTALLED_CONTROL_V11_SHA,INSTALLED_CONTROL_V12_SHA,INSTALLED_CONTROL_V13_SHA,INSTALLED_CONTROL_V14_SHA,INSTALLED_CONTROL_V15_SHA,INSTALLED_CONTROL_R6_SHA,INSTALLED_CONTROL_R11_SHA)
    # A caller auditing mutable saved settings can bind its exact source snapshot.
    if accepted_source_sha is not None:
        if not isinstance(accepted_source_sha,str) or len(accepted_source_sha)!=64 or any(c not in '0123456789abcdef' for c in accepted_source_sha):
            raise ValueError('Invalid exact source pin')
        accepted=(accepted_source_sha,)
    if len(original)!=131072 or sha(original) not in accepted:
        raise ValueError("Expected valve-2 motor-off baseline differs")
    if original[0x1e00:0x1e04]!=b"\xff"*4:
        raise ValueError("Bootloader update marker is active or unknown")
    if (manifest.get("flash_application") is not True or manifest.get("automatic_startup_motion") is not False or
        manifest.get("code_address")!=BASE+START or
        manifest.get("linked_service_model_pending") is not False or
        manifest.get("bin_sha256")!=sha(image) or manifest.get("binary_bytes")!=len(image)):
        raise ValueError("Application build is not qualified or image differs")
    model=manifest.get("thumb_models",{})
    if model.get("complete") is not True or model.get("bin_sha256")!=sha(image):
        raise ValueError("Complete actual-linked application results required")
    if manifest.get("radio_calibration_enabled") is True and not {
        "calibration_cancel","calibration_duplicate","calibration_no_motion",
        "calibration_timing_temperature"
    }<=set(model.get("cases",{})):
        raise ValueError("Calibration command models are incomplete")
    if manifest.get("radio_thermostat_regulation_enabled") is True:
        control=manifest.get("normal_control_models",{})
        if (control.get("complete") is not True or control.get("bin_sha256")!=sha(image) or
            not {"off_owner","partial_then_off"}<=set(control.get("cases",{}))):
            raise ValueError("Normal radio-control results are incomplete or mismatched")
        stress=manifest.get('sampling_stress_models',{})
        if (manifest.get('critical_motor_sampling_isolated') is not True or
            stress.get('complete') is not True or stress.get('all_clean') is not True or
            stress.get('bin_sha256')!=sha(image) or
            {case.get('instruction_cost_cycles') for case in stress.get('cases',[])}!={2,3,4} or
            any(case.get('driver_error')!=0 or case.get('final_driver_error')!=0 or
                case.get('first_windows_samples',0)<3 for case in stress.get('cases',[]))):
            raise ValueError('Critical motor sampling stress results are incomplete or mismatched')
        wire=manifest.get('uart_wire_models',{})
        if (wire.get('complete') is not True or wire.get('all_clean') is not True or
            wire.get('bin_sha256')!=sha(image) or set(wire.get('cases',{}))!={'calibration','sensor'} or
            any(case.get('clean') is not True or case.get('overruns')!=0 or
                case.get('one_byte_receive_register') is not True or
                case.get('idle_fast_forward_yields_at_arrivals') is not True
                for case in wire.get('cases',{}).values())):
            raise ValueError('Finite UART receive-register results are incomplete or mismatched')
    if not 192<=len(image)<=END-START or len(image)%4:
        raise ValueError("Application crosses the active-image boundary")
    sp,entry=struct.unpack_from("<II",image)
    if (sp!=0x20007000 or not entry&1 or not BASE+START+192<=entry<BASE+START+len(image) or
        entry!=manifest.get("entry") or manifest.get("symbols",{}).get("w600_vectors")!=BASE+START):
        raise ValueError("Application vectors differ")
    final=bytearray(original)
    final[START:START+len(image)]=image
    pages=[BASE+offset for offset in range(START,START+len(image),PAGE)
           if original[offset:offset+PAGE]!=final[offset:offset+PAGE]]
    if not pages:
        raise ValueError("Application image is unchanged")
    pages=sorted((p for p in pages if p!=BASE+START),reverse=True)+([BASE+START] if BASE+START in pages else [])
    assert final[:START]==original[:START] and final[START+len(image):]==original[START+len(image):]
    assert all(BASE+START<=p and p+PAGE<=BASE+END for p in pages)
    return bytes(final),pages


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("original",type=Path)
    parser.add_argument("build",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    original=args.original.read_bytes()
    image=(args.build/"replacement-core.bin").read_bytes()
    manifest_bytes=(args.build/"manifest.json").read_bytes()
    final,pages=image_plan(original,image,json.loads(manifest_bytes))
    args.output.mkdir(exist_ok=False)
    (args.output/"final.bin").write_bytes(final)
    (args.output/"plan.json").write_text(json.dumps({
        "original_sha256":sha(original),"final_sha256":sha(final),
        "build_manifest_sha256":sha(manifest_bytes),"image_sha256":sha(image),
        "image_bytes":len(image),"pages_in_order":pages,
        "page_tail_preserved":True,"device_access":False,
        "power_loss_atomic":False,"installation_qualified":False,
        "normal_controls_complete":False},indent=2)+"\n")


if __name__=="__main__": main()
