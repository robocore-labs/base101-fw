"""Server trial integration, exclusively against a test PTY."""
import json
import os
from pathlib import Path
import pty
import subprocess
import tempfile
import threading
import time
import unittest
from calibration_emulator import TelemetryLink,server
class TrialServerTest(unittest.TestCase):
 def setUp(self):
  master,self.slave=pty.openpty();self.process=subprocess.Popen(['/tmp/base101-calibration-protocol-test','--emulate'],stdin=master,stdout=master);os.close(master)
  self.storage=tempfile.TemporaryDirectory();self.link=TelemetryLink(os.ttyname(self.slave),self.storage.name);self.seq=0
  self.wait(lambda:self.link.snapshot()['connected'])
 def tearDown(self):
  self.link.close();os.close(self.slave);self.process.terminate();self.process.wait(timeout=2);self.storage.cleanup()
 def wait(self,predicate,timeout=5):
  end=time.monotonic()+timeout
  while not predicate() and time.monotonic()<end:time.sleep(.01)
  self.assertTrue(predicate(),self.link.snapshot())
 def command(self,action,**kwargs):
  self.seq+=1;return self.link.operator(action,dict(client='test_trial_operator',seq=self.seq,**kwargs))
 def start(self,recipe='forward',**kwargs):
  self.command('arm');self.command('trial',recipe=recipe,setup_confirmed=True,**kwargs)
 def test_presence_is_distinct_from_heartbeat(self):
  self.start();self.wait(lambda:self.link.snapshot()['status']['motion_enabled']);self.wait(lambda:self.link.snapshot()['trial'] is None)
  self.wait(lambda:not self.link.snapshot()['status']['motion_enabled']);self.assertFalse(self.link.results[-1]['complete']);self.assertTrue(self.link.snapshot()['connected'])
 def test_complete_review_and_persist(self):
  self.start();deadline=time.monotonic()+6
  while self.link.snapshot()['trial'] is not None and time.monotonic()<deadline:
   try:self.command('presence')
   except ValueError:break
   time.sleep(.05)
  self.wait(lambda:self.link.snapshot()['trial'] is None);result=self.link.results[-1]
  self.assertTrue(result['complete']);self.assertTrue(result['analysis']['valid'],result['analysis'])
  self.link.review_trial(dict(id=result['id'],observation='good',notes='direction confirmed'))
  path=Path(self.storage.name)/(result['id']+'.json')
  self.wait(lambda:path.exists() and json.loads(path.read_text()).get('observation')=='good')
  exported=self.link.trial_data()['results'][-1];self.assertNotIn('samples',exported);self.assertEqual(exported['notes'],'direction confirmed')
  self.link.close();self.link=TelemetryLink(os.ttyname(self.slave),self.storage.name)
  self.wait(lambda:self.link.snapshot()['connected'])
  self.assertEqual(self.link.results[-1]['notes'],'direction confirmed')
  self.assertIsNone(self.link.snapshot()['trial'])
 def test_verification_allows_pi_and_releases_at_yaw_target(self):
  original=self.link._exchange
  def with_pi(fd,command):
   response=original(fd,command)
   if response.get('ok'):
    response['yaw_feedback']=True;response['parameters']['yaw_feedback']=1
    with self.link.lock:self.link.latest=response
   return response
  self.link._exchange=with_pi
  self.wait(lambda:self.link.snapshot()['status']['parameters']['yaw_feedback']==1)
  self.start('verify-turn-1',surface='wood')
  self.wait(lambda:self.link.snapshot()['status']['motion_enabled'])
  # Fast-forward only the test fixture's measurement clock to just before 360°.
  self.link._measured_at=time.monotonic()-15.65
  deadline=time.monotonic()+4
  while self.link.snapshot()['trial'] is not None and time.monotonic()<deadline:
   try:self.command('presence')
   except ValueError:break
   time.sleep(.05)
  self.wait(lambda:self.link.snapshot()['trial'] is None)
  r=self.link.results[-1];self.assertTrue(r['complete'],r['reason']);self.assertTrue(r['analysis']['valid'],r['analysis'])
  self.assertLess(r['elapsed_s'],4)
 def shortened_step(self):
  from unittest.mock import patch
  import contextlib,copy
  @contextlib.contextmanager
  def shortened():
   originals={rid:server.BY_ID[rid] for rid in ('forward','backward')}
   recipes=[]
   for rid,r in originals.items():
    r=copy.deepcopy(r);r['duration_s']=.35;r['stages'][0]['duration_s']=.35;recipes.append(r)
   with patch.object(server,'RECIPES',recipes),patch.dict(server.BY_ID,{r['id']:r for r in recipes}):yield
  return shortened()
 def pump_sequence(self,timeout=5):
  end=time.monotonic()+timeout
  while self.link.snapshot()['sequence'] and time.monotonic()<end:
   try:self.command('presence')
   except ValueError:break
   time.sleep(.04)
  self.wait(lambda:self.link.snapshot()['sequence'] is None)
 def test_sequence_advances_without_rearming_and_reviews_once(self):
  with self.shortened_step():
   self.command('arm');self.command('sequence',step=0,setup_confirmed=True)
   self.pump_sequence()
   self.assertTrue(self.link.snapshot()['last_sequence']['complete'],self.link.snapshot()['last_sequence'])
   self.assertEqual([r['recipe'] for r in self.link.results],['forward','backward'])
   self.assertTrue(all(r['complete'] for r in self.link.results))
   self.assertFalse(self.link.snapshot()['operator']['armed'])
   result=self.link.results[-1]
   self.link.review_trial(dict(id=result['id'],sequence_id=result['sequence_id'],observation='good'))
   self.assertTrue(all(r['observation']=='good' for r in self.link.results))
 def test_stop_cancels_remaining_subtests(self):
  with self.shortened_step():
   self.command('arm');self.command('sequence',step=0,setup_confirmed=True)
   self.wait(lambda:self.link.snapshot()['status']['motion_enabled'])
   self.link.cancel_operator();self.link.request('stop');time.sleep(.8)
   self.assertIsNone(self.link.snapshot()['sequence']);self.assertEqual(len(self.link.results),1)
   self.assertFalse(self.link.results[0]['complete']);self.assertFalse(self.link.snapshot()['status']['motion_enabled'])
 def test_trial_presence_tolerates_browser_jitter(self):
  self.start();self.wait(lambda:self.link.snapshot()['status']['motion_enabled'])
  time.sleep(.4)
  self.assertIsNotNone(self.link.snapshot()['trial'])
  self.command('presence');time.sleep(.4)
  self.assertIsNotNone(self.link.snapshot()['trial'])
  self.wait(lambda:self.link.snapshot()['trial'] is None)
  self.assertIn('browser keepalive',self.link.results[-1]['reason'])
 def test_telemetry_timeout_has_separate_reason(self):
  with self.link.lock:
   self.link.armed=True;self.link.intent_until=time.monotonic()+2
   self.link.received=time.monotonic()-1.1
   self.assertEqual(self.link._presence_failure_locked(time.monotonic()),'MCU telemetry expired (1 s)')
 def test_presence_expiry_between_subtests_aborts_sequence(self):
  with self.shortened_step():
   self.command('arm');self.command('sequence',step=0,setup_confirmed=True)
   while self.link.snapshot()['trial']:
    self.command('presence');time.sleep(.04)
   with self.link.lock:self.link.sequence['resume_at']=time.monotonic()+1.5
   self.wait(lambda:self.link.snapshot()['sequence'] is None)
   self.assertFalse(self.link.snapshot()['last_sequence']['complete']);self.assertFalse(self.link.snapshot()['operator']['armed'])
 def test_runtime_config_order_and_bound_validation(self):
  config=dict(self.link.snapshot()['status']['parameters']);config.update(radius=.04,separation=.3,ramp=2)
  validated=server.validate_parameters(dict(reversed(list(config.items()))))
  reply=self.link.request('config '+' '.join(format(validated[k],'.9g') for k in server.LIMITS))
  self.assertEqual(reply['parameters'],config)
  with self.assertRaises(ValueError):server.validate_parameters(dict(config,ramp=1.2))
  self.command('arm')
  with self.assertRaises(ValueError):self.command('trial',recipe='spin-0.2-CCW-1',setup_confirmed=True)
  reply=self.link.request('config '+' '.join(format(dict(config,yaw_feedback=1)[k],'.9g') for k in server.LIMITS))
  with self.assertRaises(ValueError):self.command('trial',recipe='forward',setup_confirmed=True)
 def test_manual_takeover_aborts_without_restarting(self):
  self.start();self.wait(lambda:self.link.snapshot()['status']['motion_enabled'])
  with self.assertRaises(ValueError):self.command('drive',vx=.1,wz=0)
  self.wait(lambda:not self.link.snapshot()['status']['motion_enabled']);self.assertFalse(self.link.results[-1]['complete']);self.assertIsNone(self.link.snapshot()['trial'])
if __name__=='__main__':unittest.main()
