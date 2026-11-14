import copy
import struct
import unittest
from unittest.mock import patch

import flash_plan as plan


class FlashPlanTests(unittest.TestCase):
    def setUp(self):
        self.original=bytes((n*7+3)&255 for n in range(131072))
        old=bytearray(self.original); old[0x1e00:0x1e04]=b"\xff"*4
        self.original=bytes(old)
        self.image=struct.pack("<II",0x20007000,0x080020c1)+b"\x00"*22680
        self.manifest={"flash_application":True,"automatic_startup_motion":False,
            "code_address":0x08002000,"linked_service_model_pending":False,
            "bin_sha256":plan.sha(self.image),"binary_bytes":len(self.image),
            "thumb_models":{"complete":True,"bin_sha256":plan.sha(self.image)},
            "entry":0x080020c1,"symbols":{"w600_vectors":0x08002000}}
        self.pin=patch.object(plan,"BASELINE_SHA",plan.sha(self.original)); self.pin.start()
        self.addCleanup(self.pin.stop)

    def test_exact_image_and_tail(self):
        final,pages=plan.image_plan(self.original,self.image,self.manifest)
        self.assertEqual(final[0x2000:0x2000+len(self.image)],self.image)
        self.assertEqual(final[:0x2000],self.original[:0x2000])
        self.assertEqual(final[0x2000+len(self.image):],self.original[0x2000+len(self.image):])
        self.assertEqual(pages[-1],0x08002000)
        self.assertEqual(pages[:-1],sorted(pages[:-1],reverse=True))
        self.assertEqual(len(pages),45)
        self.assertTrue(all(0x08002000<=p<0x0800ee00 for p in pages))

    def test_exact_audited_source_override(self):
        changed=bytearray(self.original);changed[0xee20]^=1;changed=bytes(changed)
        digest=plan.sha(changed)
        with self.assertRaises(ValueError):plan.image_plan(changed,self.image,self.manifest)
        final,_=plan.image_plan(changed,self.image,self.manifest,accepted_source_sha=digest)
        self.assertEqual(final[0xee00:],changed[0xee00:])
        for bad in (None,'','0'*64,digest.upper(),123):
            with self.assertRaises(ValueError):
                plan.image_plan(changed,self.image,self.manifest,accepted_source_sha=bad)
        with self.assertRaises(ValueError):
            plan.image_plan(self.original,self.image,self.manifest,accepted_source_sha=digest)
        marked=bytearray(changed);marked[0x1e00]=0
        with self.assertRaises(ValueError):
            plan.image_plan(bytes(marked),self.image,self.manifest,accepted_source_sha=plan.sha(marked))

    def test_reject_unqualified_or_mismatched(self):
        for key,value in (("flash_application",False),("automatic_startup_motion",True),
                          ("code_address",0x20001000),("linked_service_model_pending",True),
                          ("bin_sha256","0"*64),("binary_bytes",len(self.image)+4),
                          ("entry",0x080020c0),("symbols",{}),
                          ("thumb_models",{"complete":False}),
                          ("thumb_models",{"complete":True,"bin_sha256":"0"*64})):
            bad=copy.deepcopy(self.manifest); bad[key]=value
            with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,bad)
        for offset in (0,0x1e00,0x2000,0x1f400):
            bad=bytearray(self.original); bad[offset]^=1
            with self.assertRaises(ValueError): plan.image_plan(bytes(bad),self.image,self.manifest)
        for key in ("flash_application","automatic_startup_motion","linked_service_model_pending"):
            bad=copy.deepcopy(self.manifest); del bad[key]
            with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,bad)

    def test_sizes_and_vectors(self):
        for image in (self.image[:188],self.image+b"x",b"x"*0xce04,
                      struct.pack("<I",0x20006000)+self.image[4:],
                      self.image[:4]+struct.pack("<I",0x08001001)+self.image[8:]):
            manifest=copy.deepcopy(self.manifest)
            manifest.update(bin_sha256=plan.sha(image),binary_bytes=len(image))
            manifest["thumb_models"]["bin_sha256"]=plan.sha(image)
            with self.assertRaises(ValueError): plan.image_plan(self.original,image,manifest)

    def test_enabled_control_paths_need_their_own_results(self):
        m=copy.deepcopy(self.manifest); m["radio_calibration_enabled"]=True
        with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,m)
        m["thumb_models"]["cases"]={n:{} for n in
            ("calibration_cancel","calibration_duplicate","calibration_no_motion",
             "calibration_timing_temperature")}
        missing=copy.deepcopy(m); del missing["thumb_models"]["cases"]["calibration_timing_temperature"]
        with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,missing)
        plan.image_plan(self.original,self.image,m)
        m["radio_thermostat_regulation_enabled"]=True
        for result in ({},{"complete":False},{"complete":True,"bin_sha256":"0"*64},
                       {"complete":True,"bin_sha256":plan.sha(self.image),"cases":{"off_owner":{}}}):
            m["normal_control_models"]=result
            with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,m)
        m["normal_control_models"]={"complete":True,"bin_sha256":plan.sha(self.image),
            "cases":{"off_owner":{},"partial_then_off":{}}}
        with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,m)
        m['critical_motor_sampling_isolated']=True
        m['sampling_stress_models']={'complete':True,'all_clean':True,'bin_sha256':plan.sha(self.image),
            'cases':[{'instruction_cost_cycles':cost,'driver_error':0,'final_driver_error':0,'first_windows_samples':3}
                     for cost in (2,3,4)]}
        with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,m)
        m['uart_wire_models']={'complete':True,'all_clean':True,'bin_sha256':plan.sha(self.image),
            'cases':{name:{'clean':True,'overruns':0,'one_byte_receive_register':True,
                           'idle_fast_forward_yields_at_arrivals':True} for name in ('calibration','sensor')}}
        plan.image_plan(self.original,self.image,m)
        for key,value in [('complete',False),('all_clean',False),('bin_sha256','0'*64),('cases',[])]:
            bad=copy.deepcopy(m);bad['sampling_stress_models'][key]=value
            with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,bad)
        bad=copy.deepcopy(m);bad['sampling_stress_models']['cases'][1]['driver_error']=5
        with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,bad)
        for key,value in [('complete',False),('all_clean',False),('bin_sha256','0'*64),('cases',{})]:
            bad=copy.deepcopy(m);bad['uart_wire_models'][key]=value
            with self.assertRaises(ValueError): plan.image_plan(self.original,self.image,bad)

    def test_explicit_second_installed_baseline(self):
        other=bytearray(self.original);other[0x2400]^=1;other=bytes(other)
        with self.assertRaises(ValueError): plan.image_plan(other,self.image,self.manifest)
        for name in ('INSTALLED_CONTROL_V2_SHA','INSTALLED_CONTROL_V8_SHA','INSTALLED_CONTROL_V11_SHA','INSTALLED_CONTROL_V12_SHA','INSTALLED_CONTROL_V13_SHA','INSTALLED_CONTROL_V14_SHA','INSTALLED_CONTROL_V15_SHA'):
            with patch.object(plan,name,plan.sha(other)):
                final,_=plan.image_plan(other,self.image,self.manifest)
                self.assertEqual(final[0x2000+len(self.image):],other[0x2000+len(self.image):])

    def test_exact_real_v14_source_pin(self):
        from pathlib import Path
        original=(Path(__file__).parent/'normal-control-host/live-v14/calibration-2/before-flash.bin').read_bytes()
        self.assertEqual(plan.sha(original),plan.INSTALLED_CONTROL_V14_SHA)
        final,_=plan.image_plan(original,self.image,self.manifest)
        self.assertEqual(final[:0x2000],original[:0x2000])
        self.assertEqual(final[0x2000+len(self.image):],original[0x2000+len(self.image):])
        changed=bytearray(original);changed[0x955c]^=1
        with self.assertRaises(ValueError):plan.image_plan(changed,self.image,self.manifest)
        unqualified=copy.deepcopy(self.manifest);unqualified['linked_service_model_pending']=True
        with self.assertRaises(ValueError):plan.image_plan(original,self.image,unqualified)

    def test_data_only_update_preserves_unchanged_activation_page(self):
        original=bytearray(self.original)
        original[0x2000:0x2000+len(self.image)]=self.image
        image=bytearray(self.image);image[0x834]=1
        m=copy.deepcopy(self.manifest)
        m['bin_sha256']=m['thumb_models']['bin_sha256']=plan.sha(image)
        with patch.object(plan,'BASELINE_SHA',plan.sha(original)):
            final,pages=plan.image_plan(bytes(original),bytes(image),m)
            self.assertEqual(pages,[0x08002800])
            self.assertEqual(final[:0x2800],original[:0x2800])
            self.assertEqual(final[0x2a00:],original[0x2a00:])
            with self.assertRaises(ValueError):
                plan.image_plan(bytes(original),self.image,self.manifest)

    def test_exact_real_v15_source_pin(self):
        from pathlib import Path
        original=(Path(__file__).parent/'normal-control-host/live-v15/calibration-1/full-trace/flash.bin').read_bytes()
        self.assertEqual(plan.sha(original),plan.INSTALLED_CONTROL_V15_SHA)
        final,_=plan.image_plan(original,self.image,self.manifest)
        self.assertEqual(final[:0x2000],original[:0x2000])
        self.assertEqual(final[0x2000+len(self.image):],original[0x2000+len(self.image):])
        changed=bytearray(original);changed[0x9624]^=1
        with self.assertRaises(ValueError):plan.image_plan(changed,self.image,self.manifest)

    def test_exact_real_r6_source_pin(self):
        from pathlib import Path
        original=(Path(__file__).parent/'calibration-report-r6-host/live-install-2/final-flash.bin').read_bytes()
        self.assertEqual(plan.sha(original),plan.INSTALLED_CONTROL_R6_SHA)
        final,_=plan.image_plan(original,self.image,self.manifest)
        self.assertEqual(final[:0x2000],original[:0x2000])
        self.assertEqual(final[0x2000+len(self.image):],original[0x2000+len(self.image):])
        self.assertEqual(final[0xee00:0xf200],original[0xee00:0xf200])
        changed=bytearray(original);changed[0xee20]^=1
        with self.assertRaises(ValueError):plan.image_plan(changed,self.image,self.manifest)


if __name__=="__main__": unittest.main()
