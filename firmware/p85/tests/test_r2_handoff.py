import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
import inspect_r2_handoff as r2

class R2HandoffTests(unittest.TestCase):
    def snapshot(self, heartbeat=10):
        data=bytearray(r2.SIZE)
        words=[0x50324331,1,2,0,2,heartbeat,0,0,0,1,1,0,0,0,0,0]
        data[64:128]=struct.pack("<16I",*words)
        data[128:132]=b"code"
        return data
    def inspect(self, first, second):
        with patch.object(r2,"regions_from_elf",return_value=[("code",r2.BASE+128,b"code")]),patch.object(r2,"report_address",return_value=r2.BASE+64):
            return r2.inspect(b"elf",first,second)
    def test_advancing_heartbeat_and_wrap(self):
        for a,b in ((10,35),(0xfffffffe,1)):
            self.assertEqual(self.inspect(self.snapshot(a),self.snapshot(b))["status"],"passed")
    def test_stale_or_backward_counter_rejected(self):
        for a,b in ((10,10),(10,9)):
            self.assertFalse(self.inspect(self.snapshot(a),self.snapshot(b))["checks"]["heartbeat_advances"])
    def test_ownership_and_other_controllers_checked(self):
        for field,value in ((3,1),(4,1),(6,1),(7,1),(8,2),(9,0),(10,0)):
            data=self.snapshot(35)
            struct.pack_into("<I",data,64+field*4,value)
            self.assertEqual(self.inspect(self.snapshot(),data)["status"],"failed")
    def test_identity_and_reserved_corruption_rejected(self):
        for field,value in ((0,0),(1,2),(2,0),(3,2),(4,3),(6,4),(11,1),(15,1)):
            data=self.snapshot(35)
            struct.pack_into("<I",data,64+field*4,value)
            with self.subTest(field=field),self.assertRaises(ValueError):self.inspect(self.snapshot(),data)
    def test_both_images_must_match(self):
        for which in (0,1):
            data=[self.snapshot(),self.snapshot(35)]
            data[which][128]=0
            self.assertFalse(self.inspect(*data)["checks"]["image_matches"])
if __name__=="__main__":unittest.main()
