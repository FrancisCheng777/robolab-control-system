"""Mock arm + synthetic or real cameras/GELLO -> raw HDF5 -> official LeRobot reader.
Always marks episodes mock; never opens Franka or drives Robotiq.
"""
import argparse,json,os,subprocess,sys,time,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from pi05_demo.cameras import Pair,SyntheticPair
from pi05_demo.collect import Gello
from pi05_demo.episode import EpisodeWriter,inspect_episode
from pi05_demo.session import record_frame
from pi05_demo.transport import Bridge
from pi05_demo.teleop_reference import j7_offset_from_config
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--real-sensors',action='store_true');args=p.parse_args()
run=ROOT/'reports/robolab_restore_20261030'/('smoke_'+uuid.uuid4().hex[:8]);run.mkdir(parents=True)
cfg=json.loads((ROOT/'configs/collection.robolab.json').read_text());cfg['sensor_smoke']=bool(args.real_sensors)
cameras=leader=None
try:
 cameras=Pair(cfg['cameras']) if args.real_sensors else SyntheticPair()
 if args.real_sensors:
  leader=Gello(cfg['gello_port'],j7_offset_rad=j7_offset_from_config(cfg),robolab=True)
  leader.read()
 for name in ('smoke_train','smoke_validation'):
  sock='/tmp/pi05-robolab-smoke-'+uuid.uuid4().hex[:8]+'.sock'
  log=(run/(name+'.log')).open('w')
  proc=subprocess.Popen([str(ROOT/'build/pi05_franka_bridge'),'--mock','--config',str(ROOT/'configs/bridge.robolab.mock.json'),'--socket',sock],stdout=log,stderr=log)
  bridge=writer=None
  try:
   deadline=time.monotonic()+5
   while time.monotonic()<deadline:
    try:bridge=Bridge(sock,mock=True);break
    except (OSError,RuntimeError):time.sleep(.01)
   if bridge is None:raise RuntimeError('mock arm unavailable')
   state=bridge.state()
   if not args.real_sensors:
    class Leader:
     def read(self):return np.array(state['q'])+.2, .02
    leader=Leader()
   writer=EpisodeWriter(run/'raw',mock=True,task='controller pipeline checkout',session=name,config={**cfg,'bridge_calibration':state['calibration']})
   bridge.claim();deadline=time.monotonic()
   for _ in range(20):
    record_frame(bridge,cameras,leader,writer)
    deadline+=1/15;time.sleep(max(0,deadline-time.monotonic()))
   path=writer.finish('success');writer=None
   inspect_episode(path,allow_mock=True)
  finally:
   if bridge:bridge.close()
   proc.terminate();proc.wait(timeout=5);log.close()
   if writer:writer.close_incomplete()
finally:
 if cameras:cameras.close()
 if args.real_sensors and leader:leader.agent._robot._driver.close()
subprocess.run([str(ROOT/'scripts/export.sh'),'--input',str(run/'raw'),'--output',str(run/'lerobot'),'--validation-session','smoke_validation','--allow-mock'],check=True)
manifest=json.loads((run/'lerobot/manifest.json').read_text())
report={'mock_arm':True,'real_sensors':args.real_sensors,'run':str(run),'validation':manifest['validation']}
(run/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
