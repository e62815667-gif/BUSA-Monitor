"""Offline contract tests for BUSA-1M-001 firmware v0.3. Run: python -m unittest discover -s tests -v"""
import json
import math
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parent

def validate_status(data):
    assert data["device"] == "BUSA-1M-001"
    assert data["mode"] == "manual"
    assert isinstance(data["volume_l"], (int, float)) and 0 <= data["volume_l"] <= 100
    assert math.isclose(data["level_mm"], data["volume_l"] * 8, abs_tol=0.11)
    assert isinstance(data["temperature_count"], int) and 0 <= data["temperature_count"] <= 4
    assert len(data["temperatures"]) == 4
    for i, t in enumerate(data["temperatures"], 1):
        assert t["channel"] == i
        assert isinstance(t["name"], str) and t["name"]
        assert isinstance(t["address"], str) and t["address"]
        assert isinstance(t["ok"], bool)
        if t["ok"]:
            assert isinstance(t["celsius"], (int, float))
            assert math.isfinite(t["celsius"]) and -55 <= t["celsius"] <= 125
        else:
            assert t["celsius"] is None
    return True

class FirmwareContractTest(unittest.TestCase):
    def test_all_connected(self):
        self.assertTrue(validate_status(json.loads((ROOT/"fixtures/status-ok.json").read_text())))
    def test_disconnected(self):
        self.assertTrue(validate_status(json.loads((ROOT/"fixtures/status-disconnected.json").read_text())))
    def test_bad_temperature_rejected(self):
        obj=json.loads((ROOT/"fixtures/status-ok.json").read_text())
        obj["temperatures"][0]["celsius"]=200
        with self.assertRaises(AssertionError):
            validate_status(obj)
    def test_bad_level_rejected(self):
        obj=json.loads((ROOT/"fixtures/status-ok.json").read_text())
        obj["level_mm"]=999
        with self.assertRaises(AssertionError):
            validate_status(obj)

if __name__ == "__main__":
    unittest.main()
