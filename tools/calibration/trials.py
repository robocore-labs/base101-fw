"""Bounded calibration recipes and deterministic measurement analysis."""
import math
import statistics

BASIC_LIMITS = {'ax':(.01,5),'jx':(.01,50),'aw':(.01,20),'jw':(.01,200),'k_icr':(1,5)}
LIMITS = {**BASIC_LIMITS, 'radius':(.02,.1),'separation':(.15,.5),'ramp':(1,255),
 'rpm_max':(20,200),'vx_max':(.05,.4),'wz_max':(.1,.8),'lpf':(1,80),'kp':(0,2),
 'ki':(0,2),'correction_max':(0,1),'deadband':(0,.05),'correction_accel':(.01,2),
 'correction_jerk':(.01,20),'yaw_feedback':(0,1)}
INTEGER_PARAMETERS = {'ramp','rpm_max','yaw_feedback'}

def recipe(name, step, stages, title, signs=None):
    return {'id':name,'step':step,'stages':[{'duration_s':d,'vx':v,'wz':w} for d,v,w in stages],
            'title':title,'signs':signs,'duration_s':sum(s[0] for s in stages)}
RECIPES = [
 recipe('forward',0,[(3,.08,0)],'Forward direction',[1,1,1,1]),
 recipe('backward',0,[(3,-.08,0)],'Backward direction',[-1,-1,-1,-1]),
 recipe('left',0,[(3,0,.3)],'Left turn direction',[-1,1,-1,1]),
 recipe('right',0,[(3,0,-.3)],'Right turn direction',[1,-1,1,-1]),
 recipe('minimum',0,[(3,.01,0),(3,.02,0),(3,.04,0)],'Minimum useful speed'),
 recipe('ramp',0,[(5,.38,0)],'Acceleration and release · ~100 RPM'),
 recipe('reverse',0,[(4,.15,0),(4,-.15,0)],'Linear reversal'),
 recipe('yaw-reverse',0,[(4,0,.4),(4,0,-.4)],'Turning reversal')]
for speed in [.2,.4,.6]:
 for repeat in [1,2]:
  for sign in [1,-1]:
   direction='CCW' if sign>0 else 'CW'
   RECIPES.append(recipe(f'spin-{speed}-{direction}-{repeat}',1,[(math.pi/speed+2,0,speed*sign)],
     f'{direction} · {speed:.2f} rad/s · repeat {repeat}'))
for speed in [.1,.2]:
 for sign in [1,-1]:
  RECIPES.append(recipe(f'brake-{speed}-{sign}',2,[(4,speed*sign,0)],f'Release from {speed*sign:+.2f} m/s'))
for sign in [1,-1]:
 for repeat in [1,2]:
  RECIPES.append(recipe(f'distance-{sign}-{repeat}',3,[(3,.1*sign,0)],f'Straight reference · {"forward" if sign>0 else "backward"} · repeat {repeat}'))
for repeat in range(1,11):
 r=recipe(f'verify-turn-{repeat}',3,[(2*math.pi/.4+2,0,.4)],f'360° odometry verification · turn {repeat} / 10')
 r.update(verification=True,target_yaw=2*math.pi)
 RECIPES.append(r)
BY_ID={r['id']:r for r in RECIPES}

def analyze(run):
    m=run.get('metrics') or {}; reasons=[]
    if not run.get('complete'): reasons.append(run.get('reason','interrupted'))
    if m.get('samples',0)<20: reasons.append('insufficient sensor samples')
    if m.get('gaps',0)>0: reasons.append('sensor gaps in the measurement window')
    r=BY_ID[run['recipe']]
    if r['step']==1:
        g=m.get('steady_gyro',0);e=m.get('steady_encoder',0)
        if m.get('steady_s',0)<2 or abs(g)<.3: reasons.append('insufficient steady rotation')
        if g*e<=0 or g*r['stages'][0]['wz']<=0: reasons.append('encoder / gyro direction mismatch')
        ratio=e/g if abs(g)>.001 else None
        if ratio is None or not 1<=ratio<=5: reasons.append('skid ratio outside supported range')
    else: ratio=None
    if r.get('verification'):
        g=m.get('gyro_yaw',0);e=m.get('encoder_yaw',0)
        if g<2*math.pi*.9 or g>2*math.pi*1.1: reasons.append('turn did not finish within 10% of 360°')
        if e*g<=0: reasons.append('encoder / gyro direction mismatch')
    if r['signs']:
        plateau=run.get('plateaus',[])
        means=plateau[-1]['mean_rpm'] if plateau else []
        if len(means)!=4 or any(v*s<=1 for v,s in zip(means,r['signs'])): reasons.append('wheel signs / minimum RPM did not match')
    if not run.get('surface') and r['step']>0: reasons.append('surface not recorded')
    # Dimensioned scores, not an invented 0–100 fitness score.
    return {'valid':not reasons,'reasons':reasons,'k_icr':ratio,
      'knocks':m.get('knocks'), 'accel_peak':m.get('accel_peak'),
      'tracking_rms':m.get('tracking_rms'),'response_s':m.get('response_s'),
      'stop_distance':m.get('stop_distance'),'stop_s':m.get('stop_s'),
      'minimum_speed':next((abs(p['vx']) for p in run.get('plateaus',[]) if p['samples']>=3 and all(v>1 for v in p['mean_rpm'])),None) if r['id']=='minimum' else None}

def summarize(runs):
    accepted=[r for r in runs if r.get('analysis',{}).get('valid') and r.get('observation')=='good']
    # Never combine different geometry, payload, surface or tuning snapshots.
    groups={}
    for r in accepted:
        if BY_ID[r['recipe']]['step']!=1: continue
        key=(tuple(sorted(r['parameters'].items())),r.get('surface'),r.get('payload'))
        groups.setdefault(key,{})[r['recipe']]=r
    if not groups: return {'count':0,'required':12,'suggestion':None,'message':'Complete and confirm floor trials to fit skid compensation.'}
    series=groups[key];values=[r['analysis']['k_icr'] for r in series.values()]
    median=statistics.median(values);spread=(max(values)-min(values))/median
    by_speed={};by_direction={}
    for r in series.values():
        w=BY_ID[r['recipe']]['stages'][0]['wz']
        by_speed.setdefault(str(abs(w)),[]).append(r['analysis']['k_icr'])
        by_direction.setdefault('CCW' if w>0 else 'CW',[]).append(r['analysis']['k_icr'])
    speed_medians={k:statistics.median(v) for k,v in by_speed.items()};direction_medians={k:statistics.median(v) for k,v in by_direction.items()}
    stable=spread<=.15 and len(series)==12
    return {'count':len(series),'required':12,'median':median,'spread':spread,
      'by_speed':speed_medians,'by_direction':direction_medians,'surface':key[1],'payload':key[2],
      'suggestion':{'k_icr':round(median,3)} if stable else None,
      'message':'All 12 trials agree within a 15% total range. Compare the candidate.' if stable else
        'Collect all 12 matching trials; a total range above 15% needs investigation.'}

def recommend(runs, recipe_id, parameters):
    same=[r for r in runs if r['recipe']==recipe_id and r.get('analysis',{}).get('valid') and
          r.get('observation') in ('good','tune') and r['parameters']==parameters]
    context=(same[-1].get('surface'),same[-1].get('payload')) if same else None
    same=[r for r in same if (r.get('surface'),r.get('payload'))==context]
    if len(same)<2: return {'suggestion':None,'message':'Repeat and confirm this baseline twice before comparing a candidate.'}
    recent=same[-2:];a=[r['analysis'] for r in recent]
    response=[s['response_s'] for s in a if s.get('response_s') is not None and s['response_s']>=0]
    if len(response)<2: return {'suggestion':None,'message':'The target was not reached; investigate tracking before changing shaping.'}
    axis='jw' if any(s['wz'] for s in BY_ID[recipe_id]['stages']) else 'jx'
    baseline=[r for r in runs if r['recipe']==recipe_id and r.get('analysis',{}).get('valid') and
      r.get('observation') in ('good','tune') and (r.get('surface'),r.get('payload'))==context and r['parameters'].get(axis)!=parameters.get(axis) and
      all(r['parameters'].get(k)==v for k,v in parameters.items() if k!=axis)]
    if len(baseline)>=2:
        old=baseline[-1]['parameters'];pair=[r for r in baseline if r['parameters']==old][-2:]
        times=[r['analysis'].get('response_s',-1) for r in pair]
        if len(pair)==2 and min(times)>=0:
            before=statistics.mean(times);after=statistics.mean(response)
            old_knocks=statistics.mean(r['analysis']['knocks'] for r in pair)
            new_knocks=statistics.mean(r['analysis']['knocks'] for r in recent)
            if after>before*1.25 or new_knocks>old_knocks:
                return {'suggestion':{axis:old[axis]},'message':'Candidate worsened response by >25% or increased transients. Restore the previous setting.',
                        'comparison':{'before_s':before,'after_s':after,'before_knocks':old_knocks,'after_knocks':new_knocks}}
    if max(response)>1.5:
        return {'suggestion':None,'message':'Response is already slow (>1.5 s). Do not lower jerk blindly; compare tracking and motor ramp first.'}
    if statistics.mean(s.get('knocks',0) for s in a)>=1:
        axis='jw' if any(s['wz'] for s in BY_ID[recipe_id]['stages']) else 'jx'
        candidate=round(max(LIMITS[axis][0],parameters[axis]*.8),3)
        return {'suggestion':{axis:candidate},'message':f'Compare {axis} at −20%, keeping acceleration unchanged. Repeat twice; reject if response worsens by >25%.',
         'baseline':{'knocks':statistics.mean(s['knocks'] for s in a),'response_s':statistics.mean(response)},'experimental':True}
    return {'suggestion':None,'message':'No threshold transients in the repeated baseline. Keep these settings until another test shows a problem.'}


def geometry_summary(runs,parameters):
    rows=[r for r in runs if BY_ID[r['recipe']]['step']==3 and not BY_ID[r['recipe']].get('verification') and r.get('analysis',{}).get('valid')
      and r.get('observation')=='good' and r.get('reference_m') and r['parameters']==parameters
      and abs(r['metrics'].get('distance',0))>.05]
    if not rows: return {'suggestion':None,'message':'Enter independently measured floor travel for each reference trial.'}
    key=(rows[-1].get('surface'),rows[-1].get('payload'))
    series={r['recipe']:r for r in rows if (r.get('surface'),r.get('payload'))==key}
    if len(series)<4:return {'suggestion':None,'message':'Complete two forward and two backward physical references with matching settings, surface and payload.'}
    ratios=[r['reference_m']/abs(r['metrics']['distance']) for r in series.values()]
    median=statistics.median(ratios);radius=parameters['radius']*median
    if (max(ratios)-min(ratios))/median>.1 or not .02<=radius<=.1:
        return {'suggestion':None,'message':'Distance references disagree by >10% or imply unsupported geometry. Investigate slip and measurements.'}
    return {'suggestion':{'radius':round(radius,4)},'message':'Compare this radius from all four physical references. Repeat after applying; physical separation and encoder-only odometry stay independent of gyro.'}


def verification_summary(runs):
    rows=[r for r in runs if BY_ID[r['recipe']].get('verification') and r.get('analysis',{}).get('valid') and r.get('observation')=='good']
    if not rows: return {'count':0,'suggestion':None,'message':'Complete ten 360° turns to compare accumulated encoder odometry with gyro yaw.'}
    last=rows[-1];p=last['parameters'];context=(last.get('surface'),last.get('payload'))
    series={r['recipe']:r for r in rows if r['parameters']==p and (r.get('surface'),r.get('payload'))==context}
    gyro=sum(r['metrics']['gyro_yaw'] for r in series.values());encoder=sum(r['metrics']['encoder_yaw'] for r in series.values())
    error=encoder-gyro
    return {'count':len(series),'required':10,'gyro_deg':math.degrees(gyro),'encoder_deg':math.degrees(encoder),
      'error_deg':math.degrees(error),'error_percent':100*error/gyro,'compensated_error_deg':math.degrees(encoder/p['k_icr']-gyro),
      'suggestion':None,'parameters':p,'message':f"{len(series)} / 10 turns: raw encoder odometry error {math.degrees(error):+.1f}° ({100*error/gyro:+.2f}%). Skid-divided yaw is diagnostic only; odometry remains encoder-only."}

def step_recommendations(runs,current):
    out=[]
    for step in range(4):
        rows=[r for r in runs if BY_ID[r['recipe']]['step']==step and not BY_ID[r['recipe']].get('verification')]
        if not rows: out.append({'step':step,'suggestion':None,'message':'No completed measurements for this step.'});continue
        p=rows[-1]['parameters'];matching=[r for r in rows if r['parameters']==p]
        if step==1: rec=summarize(matching)
        elif step==3: rec=geometry_summary(matching,p)
        else:
            options=[dict(recommend(rows,rid,p),recipe=rid) for rid in dict.fromkeys(r['recipe'] for r in matching)]
            rec=next((r for r in options if r.get('suggestion')),options[-1])
        out.append(dict(rec,step=step,parameters=p,historical=p!=current))
    return out
