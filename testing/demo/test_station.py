import sys
import struct
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
from demo_station import decode_ir_diag, decode_demo, start_blockers, status_line
import base_station
class DemoProtocol(unittest.TestCase):
 def test_frame(self):
  d=decode_demo(struct.pack("<16I",1,1|2|4|8|16,100,2,1,2,3,100,101,102,3000,3001,3002,0xffec|(20<<16),3<<16,100))
  self.assertEqual(d["duty_left"],-20);self.assertEqual(d["duty_right"],20)
  self.assertIsNone(d["speed_mm_s"]);self.assertIsNone(d["distance_cm"]);self.assertIsNone(d["last_hump_mm"])
  self.assertTrue(d["pickup_latched"]);self.assertEqual(d["calibration_state"],"ready")
 def test_invalid(self):
  for x in (b"",bytes(63),bytes(65),bytes(64)):
   with self.assertRaises(ValueError):decode_demo(x)
 def test_readiness(self):
  d=dict(ir_calibrated=True,ir_fresh=True,imu_ready=False)
  t=dict(state="STANDBY",imu={"imu_status":"FAULT"})
  self.assertEqual(len(start_blockers(d,t)),1)
  self.assertIn("IMU not ready",start_blockers(d,t)[0])
  self.assertIn("IMU=FAULT",status_line(d,t))
  d["imu_ready"]=True
  self.assertEqual(start_blockers(d,t),[])
  d["pickup_latched"]=True
  self.assertTrue(start_blockers(d,t))
 def test_no_imu_profile(self):
  d=decode_demo(struct.pack("<16I",1,128|2|4,0,0,0,0,0,0,0,0,0,0,0,0,0,0))
  self.assertTrue(d["imu_ignored"])
  self.assertFalse(d["imu_ready"])
  self.assertEqual(start_blockers(d,{"state":"STANDBY"}),[])
  d["ir_fresh"]=False
  self.assertIn("IR readings stale/invalid",start_blockers(d,{"state":"STANDBY"}))
 def test_no_echo_display(self):
  self.assertIn("sonar=no echo/pending", status_line({}, {"ultrasound":{"distance_cm":511}}))
  self.assertIn("sonar=30cm", status_line({}, {"ultrasound":{"distance_cm":30}}))
 def test_ir_diag(self):
  frame=struct.pack("<16I",0x31425249,*([100,1234|(2500<<16),4|8|16|(2<<5)|(12<<9)|(10<<21),20|(18<<16),42]*3))
  d=decode_ir_diag(frame)[0]
  self.assertEqual(d["raw_right"],1234)
  self.assertEqual(d["darkness"],2500)
  self.assertEqual(d["outcome"],"motion_invalid_or_changed")
  self.assertEqual(d["runs_before"],12)
  self.assertEqual(d["duty_right"],18)
 def test_stationary_bench(self):
  d=decode_demo(struct.pack("<16I",1,256|2|4|8,0,0,0,0,0,0,0,0,0,0,0,0,0,0))
  self.assertTrue(d["barcode_bench"])
  self.assertIn("START disabled",start_blockers(d,{"state":"STANDBY"})[0])
 def test_missing_telemetry(self):
  self.assertEqual(len(start_blockers({},{})),2)
  self.assertIn("IR=?",status_line({},{}))
 def test_commands(self):
  self.assertEqual(base_station.encode_command(["start"]),b"\x20")
  self.assertEqual(base_station.encode_command(["stop"]),b"\x40")
if __name__=="__main__":unittest.main()
