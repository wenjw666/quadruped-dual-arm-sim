import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
spec=importlib.util.spec_from_file_location("handoff_guard",Path(__file__).resolve().parents[1]/"tools/can_handoff_flash_guard.py")
g=importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)

class FlashGuardTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.original=bytes(range(256))*256
        self.flash=bytearray(self.original)
        self.calls=[]
        self.fail_write_once=False
        self.boot="boot-a"
        (self.root/"flash-before.bin").write_bytes(self.original)
        info={"origin_boot_id":"boot-a","candidates":{},"pinned":{}}
        for core,value in ((2,0x22),(0,0x55)):
            data=bytearray([value]*0x1000)
            struct.pack_into("<4I",data,0,0xABCD1234,500,0xffffffff,64)
            name=f"rt{core}-candidate.bin"
            (self.root/name).write_bytes(data)
            info["candidates"][str(core)]=name
            info["pinned"][name]=g.sha(data)
        raw=json.dumps(info).encode()
        (self.root/"package-info.json").write_bytes(raw)
        self.patcher=patch.multiple(g,BASE=self.root,EXPECTED=self.root,MARKER=self.root/"armed.json",
            PACKAGE_SHA256=g.sha(raw),OLD_FULL=g.sha(self.original),TOTAL=0x10000,SLOT=0x1000,
            SLOTS=((2,0x3000),(0,0x1000)),boot_id=lambda:self.boot,run=self.fake_run)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.output=contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__,None,None,None)
    def fake_run(self,*args):
        self.calls.append(args)
        if "reboot" in args:
            raise AssertionError("Reboot must never be issued")
        if args[0]!="mtd_debug":return
        action=args[1]
        offset,size=int(args[3],0),int(args[4],0)
        if action=="read":Path(args[5]).write_bytes(self.flash[offset:offset+size])
        elif action=="erase":self.flash[offset:offset+size]=b"\xff"*size
        elif action=="write":
            if self.fail_write_once:
                self.flash[offset:offset+32]=b"X"*32
                self.fail_write_once=False
                raise RuntimeError("simulated partial write failure")
            self.flash[offset:offset+size]=Path(args[5]).read_bytes()
    def test_install_and_next_boot_restore(self):
        g.install()
        self.assertNotEqual(self.flash,self.original)
        self.assertTrue(g.MARKER.exists())
        g.outside_unchanged(self.flash,self.original)
        self.boot="boot-b"
        g.restore(True)
        self.assertEqual(self.flash,self.original)
        self.assertFalse(g.MARKER.exists())
        self.assertFalse(json.loads((self.root/"rollback-result.json").read_text())["accepted"])
    def test_same_boot_automatic_restore_rejected(self):
        g.install()
        before=bytes(self.flash)
        with self.assertRaises(RuntimeError):g.restore(True)
        self.assertEqual(self.flash,before)
        self.assertTrue(g.MARKER.exists())
    def test_candidate_corruption_refuses_erase(self):
        (self.root/"rt2-candidate.bin").write_bytes(b"bad")
        with self.assertRaises(RuntimeError):g.install()
        self.assertEqual(self.flash,self.original)
        self.assertFalse(any(c[:2]==("mtd_debug","erase") for c in self.calls))
    def test_failed_write_restores_both_slots(self):
        self.fail_write_once=True
        with self.assertRaisesRegex(RuntimeError,"partial write"):g.install()
        self.assertEqual(self.flash,self.original)
        self.assertFalse(g.MARKER.exists())
    def test_restore_independent_of_candidate_and_metadata(self):
        g.install()
        for name in ("package-info.json","rt0-candidate.bin","rt2-candidate.bin"):(self.root/name).unlink()
        g.restore(False)
        self.assertEqual(self.flash,self.original)
    def test_outside_slot_change_refuses_restore(self):
        g.install()
        self.flash[0x2000]^=1  # represents protected R1 between R0 and R2
        before=bytes(self.flash)
        with self.assertRaisesRegex(RuntimeError,"outside"):g.restore(False)
        self.assertEqual(self.flash,before)
        self.assertTrue(g.MARKER.exists())
    def test_boot_change_refuses_install(self):
        self.boot="boot-b"
        with self.assertRaisesRegex(RuntimeError,"Boot changed"):g.install()
        self.assertFalse(any(c[:2]==("mtd_debug","erase") for c in self.calls))
    def test_candidate_mapping_must_be_pinned_basename(self):
        for bad in ({"0":"rt0-candidate.bin"},
                    {"0":"rt0-candidate.bin","2":"../rt2-candidate.bin"},
                    {"0":"rt0-candidate.bin","2":"unlisted.bin"}):
            raw=json.loads((self.root/"package-info.json").read_text())
            raw["candidates"]=bad
            data=json.dumps(raw).encode()
            (self.root/"package-info.json").write_bytes(data)
            with patch.object(g,"PACKAGE_SHA256",g.sha(data)):
                with self.assertRaisesRegex(RuntimeError,"Candidate"):
                    g.install()
            self.assertFalse(any(c[:2]==("mtd_debug","erase") for c in self.calls))
    def test_baseline_and_metadata_pins(self):
        for name in ("flash-before.bin","package-info.json"):
            old=(self.root/name).read_bytes()
            (self.root/name).write_bytes(b"bad")
            with self.assertRaises(RuntimeError):g.install()
            (self.root/name).write_bytes(old)
        self.assertFalse(any(c[:2]==("mtd_debug","erase") for c in self.calls))
if __name__=="__main__":unittest.main()
