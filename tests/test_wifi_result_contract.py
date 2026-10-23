"""Check scan counters and Wi-Fi web payloads without a physical ESP32.

The V14.20 hardware telemetry is a known regression: ap_count=29, records=0,
visible SSIDs=0. A valid network discovery must not be reported as a useful
list when all displayed SSIDs are missing.
"""
import json
import unittest
from pathlib import Path

SOURCE = Path("firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h").read_text()
WIFI = Path("firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_WifiRecovery.h").read_text()


def validate_wifi_scan(scan_diag, ui_data):
    networks = ui_data.get("networks")
    assert isinstance(networks, list), "missing networks array"
    assert ui_data.get("scanning") is False, "scan still in progress"
    names = [n.get("ssid") for n in networks if isinstance(n, dict) and n.get("ssid")]
    assert len(names) == len(set(names)), "duplicate SSID entries"
    if scan_diag.get("ap_count", 0) > 0 and not names:
        raise AssertionError("SCAN_RESULT_LOST: driver found APs but UI has no SSIDs")
    if scan_diag.get("visible_ssid_count") is not None:
        assert scan_diag["visible_ssid_count"] == len(names), "visible SSID counter mismatch"
    return names


class WifiResultsContract(unittest.TestCase):
    def test_known_v1420_regression_detected(self):
        with self.assertRaisesRegex(AssertionError, "SCAN_RESULT_LOST"):
            validate_wifi_scan(
                {"ap_count": 29, "record_count": 0, "visible_ssid_count": 0},
                {"scanning": False, "networks": []},
            )

    def test_25_networks_can_be_returned(self):
        networks = [{"ssid": f"AP-{i:02}", "rssi": -40-i} for i in range(25)]
        names = validate_wifi_scan(
            {"ap_count": 25, "visible_ssid_count": 25},
            {"scanning": False, "networks": networks},
        )
        self.assertEqual(len(names), 25)

    def test_network_scan_endpoint_preserved(self):
        self.assertIn('server.on("/api/wifi/scan"', SOURCE)
        self.assertIn('wifiScanJSON()', SOURCE)
        self.assertIn('esp_wifi_scan_start(&config,true)', WIFI)
        self.assertIn('WiFi.reconnect()', WIFI)

    def test_no_unsafe_output_enabling(self):
        self.assertIn('RELAY_PHYSICAL_OUTPUTS=false,LED_PHYSICAL_OUTPUTS=false', SOURCE)
        self.assertIn('TPIN[4]={16,17,18,21}', SOURCE)


if __name__ == "__main__":
    unittest.main()
