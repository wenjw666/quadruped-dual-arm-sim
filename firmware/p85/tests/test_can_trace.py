import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from inspect_can_trace import decode,WORDS,CAPACITY

class TraceTests(unittest.TestCase):
    def sample(self):
        w=[0]*WORDS
        w[:8]=[0x50545231,1,4,0,1,1,CAPACITY,16]
        # Successful initial init, then reset error during the later START.
        for i,(stage,status) in enumerate(((7,0),(4,3),(9,1),(10,1))):
            a=8+16*i;w[a:a+4]=[stage,1,status,0]
        return w
    def unpack(self,w):
        return decode(struct.pack("<"+"I"*len(w),*w))
    def test_failed_controller_is_complete_diagnostic(self):
        r=self.unpack(self.sample())
        self.assertEqual(r["status"],"complete")
        self.assertEqual([e["stage"] for e in r["errors"]],["soft_reset","start_return"])
        self.assertEqual(r["failure"],1)
        self.assertFalse(r["can1_ownership_confirmed"])
    def test_incomplete_and_overflow_never_complete(self):
        for field,value in ((3,1),(4,0)):
            w=self.sample();w[field]=value
            self.assertEqual(self.unpack(w)["status"],"incomplete")
    def test_corruption_is_rejected(self):
        for field,value in ((0,0),(1,2),(2,CAPACITY+1),(4,2),(5,2),(6,1),(7,15),(8,99),(9,6),(20,1)):
            w=self.sample();w[field]=value
            with self.subTest(field=field),self.assertRaises(ValueError): self.unpack(w)
        with self.assertRaises(ValueError): decode(b"short")
if __name__=="__main__": unittest.main()
