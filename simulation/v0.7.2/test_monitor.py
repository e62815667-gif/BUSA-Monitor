import unittest
from monitor import Monitor

class MonitorTests(unittest.TestCase):
    def test_initial_state_is_read_only_and_safe(self):
        m=Monitor()
        t=m.telemetry()
        self.assertFalse(t["outputs_enabled"])
        self.assertEqual(t["k_states"], {"K1":False,"K2":False,"K3":False,"K4":False})
        self.assertEqual(t["temperatures"], [None]*4)
        self.assertFalse(m.capabilities()["physical_outputs"])

    def test_valid_temperatures_and_stale_rejection(self):
        m=Monitor(bridge_online=True)
        for i,v in enumerate([20,35.5,85,45]):
            m.update(i,v,100,True)
        self.assertEqual(m.telemetry()["temperatures"],[20,35.5,85,45])
        m.update(2,90,3001,True)
        self.assertIsNone(m.telemetry()["temperatures"][2])
        m.bridge_online=False
        self.assertEqual(m.telemetry()["temperatures"],[None]*4)

    def test_invalid_sensor_values(self):
        m=Monitor(bridge_online=True)
        for v in [float("nan"),float("inf"),126,-56,None]:
            m.update(0,v,100,True)
            self.assertFalse(m.telemetry()["channels"][0]["valid"])
        with self.assertRaises(ValueError): m.update(4,20,0,True)

    def test_key_journal_and_isolation(self):
        m=Monitor(keypad_online=True)
        for i in range(70): m.press(str(i))
        self.assertEqual(len(m.keys()["events"]),64)
        self.assertEqual(m.telemetry()["key_seq"],70)
        self.assertTrue(all(v is False for v in m.telemetry()["k_states"].values()))

    def test_no_actuation_from_any_input(self):
        m=Monitor(bridge_online=True,keypad_online=True)
        for t in [-55,16,30,45,65,66,85,92,125]:
            for channel in range(4): m.update(channel,t,0,True)
            m.press("K1")
            self.assertFalse(m.telemetry()["outputs_enabled"])
            self.assertTrue(all(v is False for v in m.telemetry()["k_states"].values()))

if __name__ == "__main__":
    unittest.main()
