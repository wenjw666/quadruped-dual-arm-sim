"""Reject incompatible/corrupt startup records before interpreting board state."""
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import inspect_aon_state as aon


def snapshot(name=b"realtime0", status=0, reason=1, count=1):
    data = bytearray(aon.SIZE)
    struct.pack_into("<II", data, aon.HEADER_OFFSET, aon.MAGIC, 0)
    data[aon.HEADER_OFFSET + 8:aon.HEADER_OFFSET + 8 + aon.RECORD.size] = aon.RECORD.pack(
        1, name, status, 0, 0, reason, count, bytes(12), 2)
    return data


class AonStateTests(unittest.TestCase):
    def test_default_does_not_mean_normal(self):
        rt0 = aon.inspect(snapshot())["rt0"]
        self.assertEqual((rt0["state"], rt0["state_name"]), (0, "default"))
        self.assertEqual((rt0["start_reason_name"], rt0["start_count"]), ("cold", 1))

    def test_normal_and_reboot_count(self):
        rt0 = aon.inspect(snapshot(status=4, reason=4, count=255))["rt0"]
        self.assertEqual((rt0["state_name"], rt0["start_reason_name"], rt0["start_count"]),
                         ("normal", "reboot", 255))

    def test_error_and_unknown_values_stay_distinct(self):
        self.assertEqual(aon.inspect(snapshot(status=65535))["rt0"]["state_name"], "error")
        rt0 = aon.inspect(snapshot(status=42, reason=42))["rt0"]
        self.assertEqual((rt0["state_name"], rt0["start_reason_name"]), ("unknown", "unknown"))

    def test_truncated_or_oversize(self):
        for size in (0, aon.SIZE - 1, aon.SIZE + 1):
            with self.subTest(size=size), self.assertRaises(ValueError):
                aon.inspect(bytes(size))

    def test_bad_magic_or_version(self):
        for field in (0, 4):
            data = snapshot()
            struct.pack_into("<I", data, aon.HEADER_OFFSET + field, 99)
            with self.subTest(field=field), self.assertRaises(ValueError):
                aon.inspect(data)

    def test_bad_markers(self):
        for field in (0, aon.RECORD.size - 2):
            data = snapshot()
            struct.pack_into("<H", data, aon.HEADER_OFFSET + 8 + field, 99)
            with self.subTest(field=field), self.assertRaises(ValueError):
                aon.inspect(data)

    def test_invalid_names(self):
        for name in (b"", b"x" * 16, b"rt\xff", b"rt\x01"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                aon.inspect(snapshot(name=name))

    def test_duplicate_domain(self):
        data = snapshot()
        start = aon.HEADER_OFFSET + 8
        data[start + aon.RECORD.size:start + aon.RECORD.size * 2] = data[start:start + aon.RECORD.size]
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            aon.inspect(data)

    def test_missing_rt0(self):
        with self.assertRaisesRegex(ValueError, "no realtime0"):
            aon.inspect(snapshot(name=b"realtime1"))

    def test_holes_keep_record_index(self):
        data = snapshot()
        start = aon.HEADER_OFFSET + 8
        data[start + aon.RECORD.size * 22:start + aon.RECORD.size * 23] = data[start:start + aon.RECORD.size]
        data[start:start + aon.RECORD.size] = bytes(aon.RECORD.size)
        self.assertEqual(aon.inspect(data)["rt0"]["index"], 22)


if __name__ == "__main__":
    unittest.main()
