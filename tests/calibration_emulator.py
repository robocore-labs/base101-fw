"""Test-only hardware telemetry around the real portable C protocol on a PTY."""
import importlib.util
from pathlib import Path
import time
spec=importlib.util.spec_from_file_location('calibration_server_fixture',Path(__file__).resolve().parents[1]/'tools/calibration/server.py')
server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)
class TelemetryLink(server.SerialLink):
    def __init__(self,*args,**kwargs):
        self._measured_at=time.monotonic();self._velocity=(0.,0.);self._last_nonzero=(0.,0.)
        super().__init__(*args,**kwargs)
    def _exchange(self,fd,command):
        response=super()._exchange(fd,command)
        if not response.get('ok'):return response
        if command.startswith('measure'):self._measured_at=time.monotonic();self._last_nonzero=(0.,0.)
        if command.startswith('drive '):
            self._velocity=tuple(float(v) for v in command.split()[1:]);self._last_nonzero=self._velocity
        if command in ('release','stop') or not response['motion_enabled']:self._velocity=(0.,0.)
        response['brake_ok_mask']=0 if response['motion_enabled'] else 15
        v,w=self._velocity;p=response['parameters'];elapsed=time.monotonic()-self._measured_at
        left=(v-w*p['k_icr']*p['separation']/2)/p['radius']*60/(2*3.141592653589793)
        right=(v+w*p['k_icr']*p['separation']/2)/p['radius']*60/(2*3.141592653589793)
        steady=max(0,elapsed-2)
        response.update(gyro={'calibrated':True,'fresh':True,'elapsed_ms':15000,'samples':1500,'rejected':0,'bias':-.019,'rate':w},
          imu={'online':True,'valid':True,'sample_us':response['mcu_us'],'raw_z':w-.019,'accel':[0,0,9.80665],'temperature':29},
          motion={'requested':[v,w],'shaped':[v,w],'measured':[v,w*p['k_icr']],'measured_valid':True},
          wheels=[{'fresh':True,'online':True,'rpm':rpm,'sent':rpm,'error':0,'temperature':30,'sample_us':response['mcu_us']} for rpm in [left,right,left,right]],
          yaw_feedback=bool(p['yaw_feedback']),geometry={'radius':p['radius'],'separation':p['separation'],'gyro_z_sign':1},
          metrics={'samples':int(elapsed*208),'gaps':0,'knocks':2 if elapsed>1 and abs(self._last_nonzero[0])>.2 else 0,
          'accel_peak':1.2,'jerk_peak':40,'accel_rms':.1,'tracking_rms':.002,'response_s':.6,
          'gyro_yaw':self._last_nonzero[1]*elapsed,'encoder_yaw':self._last_nonzero[1]*elapsed*p['k_icr'],
          'distance':self._last_nonzero[0]*elapsed,'steady_s':steady,'steady_gyro':self._last_nonzero[1]*steady,
          'steady_encoder':self._last_nonzero[1]*steady*p['k_icr'],'stop_distance':.03,'stop_s':.4},
          odom={'valid':True,'x':v*elapsed,'y':0,'yaw':w*elapsed*p['k_icr']})
        with self.lock:self.latest=response
        return response
