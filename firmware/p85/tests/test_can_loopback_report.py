import struct
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from inspect_can_loopback import decode, WORDS

class LoopbackAcceptance(unittest.TestCase):
    def valid(self):
        words=[0x504c4231,1,2,0,512,512,512,512,0,0,0,0,0,10240,1]+[32]*16
        normal=[0x20000,0,0x80139a73,0x31c63,0x80000000|0x8a00,0,0,0]
        loop=normal.copy();loop[0]=0;loop[1]=0x1000;loop[4]&=~0x8000
        return words+normal+loop+normal
    def check(self,words):
        return decode(struct.pack("<"+"I"*WORDS,*words))["status"]
    def test_success(self):
        self.assertEqual(self.check(self.valid()),"passed")
    def test_incomplete_or_corrupt_results_rejected(self):
        for index in (0,1,2,3,4,5,6,7,8,9,10,11,12,14,15,30):
            with self.subTest(word=index):
                words=self.valid();words[index]^=1
                self.assertEqual(self.check(words),"failed")
    def test_wrong_mode_rate_or_fd_settings_rejected(self):
        for index,mask in ((31,0x20000),(32,0x1000),(39,0x20000),(40,0x1000),(40,8),(47,0x20000),(48,8),(33,1),(42,1),(50,1),(43,0x80000000),(51,0x8000)):
            with self.subTest(word=index,mask=mask):
                words=self.valid();words[index]^=mask
                self.assertEqual(self.check(words),"failed")
    def test_read_only_tdc_measurement_does_not_fail_restore(self):
        words=self.valid();words[43]^=0x403f;words[51]^=0x4011
        self.assertEqual(self.check(words),"passed")
    def test_tdc_enabled_in_loopback_rejected(self):
        words=self.valid();words[43]|=0x8000
        self.assertEqual(self.check(words),"failed")
    def test_unrealistic_elapsed_rejected(self):
        words=self.valid();words[13]=0
        self.assertEqual(self.check(words),"failed")
    def test_short_report_rejected(self):
        with self.assertRaises(ValueError):decode(bytes(WORDS*4-4))

if __name__=="__main__":unittest.main()
