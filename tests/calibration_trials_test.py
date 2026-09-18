import copy
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/calibration'))
import trials

PARAMS={name:(lo+hi)/2 for name,(lo,hi) in trials.LIMITS.items()}
PARAMS.update(ax=.7,jx=2,aw=3,jw=10,k_icr=1.5,radius=.0363,separation=.2899,ramp=1,rpm_max=200,vx_max=.4,wz_max=.8,lpf=20,yaw_feedback=0)
def result(recipe='ramp',ratio=1.5,params=None):
 r={'id':recipe,'recipe':recipe,'parameters':copy.deepcopy(params or PARAMS),'complete':True,'reason':'completed',
  'surface':'wood','payload':'normal','observation':'good','metrics':{'samples':1000,'gaps':0,'knocks':2,
  'steady_s':4,'steady_gyro':1,'steady_encoder':ratio,'response_s':.6,'distance':.3,'stop_distance':.04,'stop_s':.5},
  'plateaus':[{'mean_rpm':[20,20,20,20]}]}
 if trials.BY_ID[recipe]['step']==1 and trials.BY_ID[recipe]['stages'][0]['wz']<0:
  r['metrics']['steady_gyro']=-1;r['metrics']['steady_encoder']=-ratio
 r['analysis']=trials.analyze(r);return r
class TrialAnalysisTest(unittest.TestCase):
 def test_recipes_bounded(self):
  self.assertEqual(len([r for r in trials.RECIPES if r['step']==1]),12)
  self.assertTrue(all(r['duration_s']+3<30 for r in trials.RECIPES))
 def test_invalid_measurements(self):
  r=result('spin-0.2-CCW-1');self.assertTrue(r['analysis']['valid'])
  r['metrics']['gaps']=1;self.assertFalse(trials.analyze(r)['valid'])
  r['metrics']['gaps']=0;r['metrics']['steady_encoder']=-1.5;self.assertFalse(trials.analyze(r)['valid'])
  r=result('forward');r['plateaus'][0]['mean_rpm'][1]=-20;self.assertFalse(trials.analyze(r)['valid'])
 def test_floor_fit_does_not_multiply_existing_icr(self):
  runs=[result(r['id'],1.5) for r in trials.RECIPES if r['step']==1]
  self.assertIsNone(trials.summarize(runs[:11])['suggestion'])
  summary=trials.summarize(runs);self.assertEqual(summary['suggestion'],{'k_icr':1.5})
  runs[-1]['metrics']['steady_encoder']*=1.5;runs[-1]['analysis']=trials.analyze(runs[-1])
  self.assertIsNone(trials.summarize(runs)['suggestion'])
 def test_do_not_mix_geometry_surface_or_payload(self):
  runs=[result(r['id']) for r in trials.RECIPES if r['step']==1]
  runs[-1]['surface']='carpet';self.assertIsNone(trials.summarize(runs)['suggestion'])
 def test_repeated_baseline_and_response_guard(self):
  runs=[result()];self.assertIsNone(trials.recommend(runs,'ramp',PARAMS)['suggestion'])
  runs.append(result());self.assertEqual(trials.recommend(runs,'ramp',PARAMS)['suggestion'],{'jx':1.6})
  p=dict(PARAMS,jx=1.6);candidate=[result(params=p),result(params=p)]
  for r in candidate:r['analysis']['response_s']=.9;r['analysis']['knocks']=1
  self.assertEqual(trials.recommend(runs+candidate,'ramp',p)['suggestion'],{'jx':2})
 def test_radius_fit_needs_both_directions_and_repeats(self):
  rows=[result(r['id']) for r in trials.RECIPES if r['step']==3 and not r.get('verification')]
  for r in rows:r['reference_m']=abs(r['metrics']['distance'])*1.02
  self.assertIsNone(trials.geometry_summary(rows[:3],PARAMS)['suggestion'])
  self.assertEqual(trials.geometry_summary(rows,PARAMS)['suggestion'],{'radius':.037})
 def test_ten_turn_verification_and_historical_step_suggestions(self):
  rows=[]
  for i in range(1,11):
   r=result(f'verify-turn-{i}');r['metrics'].update(gyro_yaw=2*trials.math.pi,encoder_yaw=3*trials.math.pi)
   r['analysis']=trials.analyze(r);rows.append(r)
  v=trials.verification_summary(rows);self.assertEqual(v['count'],10);self.assertAlmostEqual(v['gyro_deg'],3600)
  self.assertAlmostEqual(v['error_percent'],50);self.assertAlmostEqual(v['compensated_error_deg'],0)
  runs=[result(r['id']) for r in trials.RECIPES if r['step']==1]
  rec=trials.step_recommendations(runs,dict(PARAMS,radius=.0374))[1]
  self.assertTrue(rec['historical']);self.assertEqual(rec['suggestion'],{'k_icr':1.5})
if __name__=='__main__':unittest.main()
