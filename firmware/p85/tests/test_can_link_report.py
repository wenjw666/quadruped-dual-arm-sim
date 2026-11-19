#!/usr/bin/env python3
import copy
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from inspect_can_link import decode, WORDS

class PhysicalReportTest(unittest.TestCase):
    def report(self):
        w=[0x504c4b31,1,2,0,2,512,2048,1,0,0,0,0,0,0]
        w+=([512,512,512]+[32]*16)*2
        registers=[0x831853,0xdb4006,0x80139a73,0x31c63,0x86dc8c00,0x519080,0,0]
        w+=registers*6
        w[92]|=0x11000000  # CAN1 stopped only after test.
        self.assertEqual(len(w),WORDS)
        return w
    def decode(self,w):
        return decode(struct.pack("<"+"I"*len(w),*w))
    def test_complete_report(self):
        self.assertEqual(self.decode(self.report())["status"],"passed")
    def test_reject_false_success(self):
        changes={"unfinished":(2,1),"wrong_phase":(4,1),"one_direction_missing":(33,0),
                 "rx_missing":(16,511),"length_missing":(51,31),"timeout":(11,1),
                 "bad_payload":(9,1),"fault_can1":(13,1),"self_reception":(52,0x811853),
                 "loopback":(69,0xdb5006),"listen_only":(61,0xdb400e),
                 "wrong_rate":(70,0x80139a72),"tdc_disabled":(72,0x86dc0c00),
                 "can1_still_stopped":(76,0x11831853),"tx_errors":(75,1),
                 "can1_not_stopped_after":(92,0x831853),"not_restored":(7,0)}
        for label,(index,value) in changes.items():
            with self.subTest(label=label):
                w=self.report();w[index]=value
                self.assertEqual(self.decode(w)["status"],"failed")
    def test_wrong_size(self):
        with self.assertRaises(ValueError): decode(b"" )
if __name__=="__main__":unittest.main()
