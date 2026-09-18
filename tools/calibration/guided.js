'use strict';
// Uses the same operator lease and 100 ms drive loop as manual control.
const wheelChecks = [
  {id:'forward', title:'Forward polarity', detail:'All four wheels should drive the robot forward. Observe the tread direction; the encoder sign alone cannot prove physical polarity.', stages:[[3,.08,0]], signs:[1,1,1,1]},
  {id:'backward', title:'Backward polarity', detail:'All four wheels should drive backward.', stages:[[3,-.08,0]], signs:[-1,-1,-1,-1]},
  {id:'left', title:'Left turn polarity', detail:'Left wheels backward, right wheels forward: counterclockwise viewed from above. The supported chassis will not turn, so ignore gyro yaw here.', stages:[[3,0,.3]], signs:[-1,1,-1,1]},
  {id:'right', title:'Right turn polarity', detail:'Left wheels forward, right wheels backward: clockwise viewed from above.', stages:[[3,0,-.3]], signs:[1,-1,1,-1]},
  {id:'minimum', title:'Low-speed sweep', detail:'Three forward targets: 0.01, 0.02 and 0.04 m/s, three seconds each. Watch for stalls, clicking or uneven rotation. This brackets useful speed in the air; floor friction may change it.', stages:[[3,.01,0],[3,.02,0],[3,.04,0]]},
  {id:'ramp', title:'Acceleration and release', detail:'Ramp toward 0.38 m/s (about 100 RPM), hold for five seconds, then release through the normal ramp. Observe vibration during acceleration and braking.', stages:[[5,.38,0]]},
  {id:'reverse', title:'Linear reversal', detail:'Forward at 0.15 m/s for four seconds, then backward for four seconds. The software profile shapes the direction change. Listen for knocks.', stages:[[4,.15,0],[4,-.15,0]]},
  {id:'yaw-reverse', title:'Turning reversal', detail:'Left at 0.4 rad/s for four seconds, then right for four seconds. Observe both sides through the reversal.', stages:[[4,0,.4],[4,0,-.4]]}
];
let guide = null, guideIndex = 0, guideResults = [], guideReview = null;
function guideVelocity() { return guide?.phase === 'drive' ? {...guide.target, held:true} : {vx:0,wz:0,held:false}; }
function cancelGuide(reason = 'Interrupted') {
  if (!guide) return;
  finishGuide(reason, false);
}
function finishGuide(reason, complete) {
  const run = guide;
  if (!run) return;
  guide = null;
  const samples = run.samples;
  const plateaus = run.check.stages.map(([duration,vx,wz], i) => {
    const start = run.check.stages.slice(0,i).reduce((sum,s)=>sum+s[0],0);
    const rows = samples.filter(s=>s.phase === 'drive' && s.elapsed >= start+duration-1 && s.elapsed < start+duration);
    return {vx,wz,samples:rows.length, mean_rpm:names.map((_,w)=>rows.length ? rows.reduce((sum,s)=>sum+s.wheels[w].rpm,0)/rows.length : null)};
  });
  const firstMotion = samples.find(s=>s.phase === 'drive' && s.wheels.every(w=>Math.abs(w.rpm)>1));
  const signs = run.check.signs;
  const signMatch = signs && plateaus[0].samples >= 3 ? plateaus[0].mean_rpm.every((rpm,i)=>rpm*signs[i] > 1) : null;
  const result = {check:run.check.id,title:run.check.title,date:new Date().toISOString(),complete,reason,
    parameters:run.parameters,geometry:run.geometry,yaw_feedback:false,plateaus,
    encoder_signs_match:signMatch,all_wheels_above_1rpm_s:firstMotion?.elapsed ?? null,release_to_rest_s:run.restTime ?? null,observation:'pending',samples};
  guideResults.push(result); guideReview = result;
  document.querySelectorAll('[data-observation]').forEach(button=>button.setAttribute('aria-pressed','false'));
  $('guide-tuning-prompt').hidden = true; $('guide-notes').value = '';
  $('guide-review').hidden = false;
  $('guide-result').textContent = `${reason}. ${signMatch === true ? 'Encoder signs match.' : signMatch === false ? 'Encoder signs differ or a wheel did not reach 1 RPM.' : 'Review the measured RPM and your observation.'} ${run.restTime == null ? '' : `Release to rest: ${run.restTime.toFixed(2)} s.`}`;
  $('guide-measurements').replaceChildren(...plateaus.map(p=> {
    const row = document.createElement('p'); row.className = 'muted small';
    row.textContent = `${p.vx.toFixed(2)} m/s · ${p.wz.toFixed(2)} rad/s → FL / FR / BL / BR: ${p.mean_rpm.map(rpm=>rpm == null ? '—' : rpm.toFixed(1)).join(' / ')} RPM (${p.samples} final-second samples)`;
    return row;
  }));
  renderGuide();
}
function renderGuide() {
  const check = wheelChecks[guideIndex];
  $('guide-title').textContent = `${guideIndex+1} / ${wheelChecks.length} · ${check.title}`;
  $('guide-detail').textContent = check.detail;
  const fresh = state?.connected && state.age_ms < 300;
  $('guide-hold').disabled = !guide && (stopBusy || !armed || !fresh || !state?.status?.ready || !state.status.at_rest || !$('wheels-lifted').checked);
  $('guide-hold').textContent = guide ? guide.phase === 'drive' ? `Running · ${Math.max(0,guide.total-(performance.now()-guide.started)/1000).toFixed(1)} s left` : 'Ramping down · keep holding' : 'Hold to run this check';
  $('guide-prev').disabled = !!guide || guideIndex === 0;
  $('guide-next').disabled = !!guide || guideIndex === wheelChecks.length-1;
  $('guide-export').disabled = !guideResults.length || armed || !!state?.status?.motion_enabled;
  $('wheels-lifted').disabled = !!guide;
  $('guide-count').textContent = `${guideResults.length} recorded checks · ${guideResults.filter(r=>r.complete && r.observation === 'good' && r.encoder_signs_match !== false).length} accepted`;
}
function guideTick() {
  if (!guide) { renderGuide(); return; }
  const run = guide, s = state?.status;
  if (!armed || !state.connected || state.age_ms >= 300 || !s?.ready || s.fault || s.yaw_feedback !== false || s.wheels?.length !== 4 || s.wheels.some(w=>!w.fresh || !Number.isFinite(w.rpm)) || JSON.stringify(s.parameters) !== JSON.stringify(run.parameters) || (run.lastMCU !== null && s.mcu_us < run.lastMCU)) {
    cancelGuide('Aborted: telemetry, ownership, parameters or readiness changed'); hardStop(false); return;
  }
  const elapsed = (performance.now()-run.started)/1000;
  if (s.mcu_us !== run.lastMCU) {
    run.lastMCU = s.mcu_us;
    run.samples.push({elapsed,phase:run.phase,mcu_us:s.mcu_us,motion:s.motion,gyro:s.gyro,wheels:s.wheels});
  }
  if (run.phase === 'drive') {
    let end = 0;
    const stage = run.check.stages.find(stage=> {end += stage[0]; return elapsed < end;});
    if (stage) run.target = {vx:stage[1],wz:stage[2]};
    else {
      run.phase = 'settling'; run.released = performance.now();
      post('/api/release',operatorPayload()).catch(error=> {notify(error.message); hardStop(false);});
    }
  } else if (s.at_rest && !s.motion_enabled) {
    run.restTime = (performance.now()-run.released)/1000;
    finishGuide('Completed', true); hardStop(false);
  } else if (performance.now()-run.released > 3000) {
    cancelGuide('Aborted: not at rest within three seconds of release'); hardStop(false);
  }
  renderGuide();
}
$('guide-hold').addEventListener('pointerdown', event=> {
  if ($('guide-hold').disabled || guide) return;
  if (keys.size || pointerDirection || state.status.yaw_feedback !== false) {notify('Release manual controls and keep yaw PI disabled before a guided check.'); return;}
  event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId);
  const check = wheelChecks[guideIndex];
  guide = {check,phase:'drive',started:performance.now(),total:check.stages.reduce((sum,s)=>sum+s[0],0),
    target:{vx:check.stages[0][1],wz:check.stages[0][2]},parameters:structuredClone(state.status.parameters),geometry:structuredClone(state.status.geometry),samples:[],lastMCU:null};
  $('guide-review').hidden = true; guideTick(); renderControl(); drive();
});
function guideReleased() {if (guide) {cancelGuide('Aborted: hold released before completion'); hardStop(false);}}
for (const event of ['pointerup','pointercancel','lostpointercapture']) $('guide-hold').addEventListener(event,guideReleased);
$('guide-prev').addEventListener('click',()=> {guideIndex--; renderGuide();});
$('guide-next').addEventListener('click',()=> {guideIndex++; renderGuide();});
$('wheels-lifted').addEventListener('change',renderGuide);
for (const button of document.querySelectorAll('[data-observation]')) {
  button.addEventListener('click',()=> {
    if (!guideReview) return;
    guideReview.observation = button.dataset.observation;
    document.querySelectorAll('[data-observation]').forEach(other=>other.setAttribute('aria-pressed',String(other === button)));
    const tune = button.dataset.observation === 'tune';
    $('guide-tuning-prompt').hidden = !tune;
    renderGuide();
    if (tune) {
      $('motion-tuning').scrollIntoView({behavior:'smooth',block:'center'});
      $('motion-tuning').focus({preventScroll:true});
    }
  });
}
$('guide-notes').addEventListener('input',()=> {if(guideReview) guideReview.notes = $('guide-notes').value;});
$('guide-export').addEventListener('click',()=>download('base101-wheel-checks.json',{format:'base101-wheel-checks-v1',date:new Date().toISOString(),checks:guideResults}));
