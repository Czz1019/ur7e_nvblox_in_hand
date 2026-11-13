#!/usr/bin/env python3
"""Fresh isolated Gazebo integration test; only stops subprocesses it starts."""
import argparse,os,signal,subprocess,time,json,traceback
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from ur7e_nvblox_interfaces.srv import FilePath,QueryEsdf
from std_srvs.srv import Trigger

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--output',default='');args=parser.parse_args()
if os.environ.get('ROS_DOMAIN_ID')!='135' or os.environ.get('GAZEBO_MASTER_URI')!='http://127.0.0.1:11565':raise RuntimeError('Simulation test requires domain135 and port11565')
out=Path(args.output).resolve() if args.output else root/'artifacts'/time.strftime('simulation_%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=False)
rclpy.init();node=Node('ur7e_python_sim_validation');procs=[];files=[];critical=[]
r={'status':'running','phases':{},'commands':[],'checks':{}};metrics=[];clouds=[];slices=[];meshes=[];depths=[];rgbs=[]
last={}
def msg_stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

def image(kind,m):last[kind]=m;(depths if kind=='depth' else rgbs).append(time.monotonic())
def pcl(kind,m):
 last[kind]=m;row={'wall':time.monotonic(),'stamp':msg_stamp(m),'points':m.width*m.height,'age_ms':(node.get_clock().now().nanoseconds-msg_stamp(m))*1e-6}
 # Collector itself uses system time; use /clock separately for simulation ages below.
 row['age_ms']=None if 'clock' not in last else (last['clock']-msg_stamp(m))*1e-6
 (clouds if kind=='cloud' else slices).append(row)
from rosgraph_msgs.msg import Clock
node.create_subscription(Clock,'/clock',lambda m:last.update(clock=m.clock.sec*10**9+m.clock.nanosec),qos_profile_sensor_data)
node.create_subscription(Image,'/camera/depth/image_rect_raw',lambda m:image('depth',m),qos_profile_sensor_data)
node.create_subscription(Image,'/camera/color/image_raw',lambda m:image('rgb',m),qos_profile_sensor_data)
node.create_subscription(PointCloud2,'/ur7e_nvblox/esdf_points',lambda m:pcl('cloud',m),1)
node.create_subscription(PointCloud2,'/ur7e_nvblox/static_esdf_pointcloud',lambda m:pcl('slice',m),1)
node.create_subscription(MarkerArray,'/ur7e_nvblox/mesh',lambda m:(last.update(mesh=m),meshes.append(time.monotonic())),1)
def metric(m):
 d=json.loads(m.data);metrics.append((time.monotonic(),d));last['metrics']=d
node.create_subscription(String,'/ur7e_nvblox/metrics',metric,10)
for layer in ['tsdf_layer','color_layer']:
 node.create_subscription(PointCloud2,'/ur7e_nvblox/'+layer,lambda m,k=layer:last.update({k:m}),1)

def start(cmd,name,required=False):
 f=(out/(name+'.log')).open('w');files.append(f);p=subprocess.Popen(cmd,cwd=root,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p)
 if required:critical.append(p)
 r['commands'].append({'name':name,'argv':cmd,'pid':p.pid});print('START',name,flush=True);return p

def stop(p):
 if p.poll() is None:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  for p in critical:
   if p.poll() is not None:raise RuntimeError('Required process exited '+str(p.pid))
  rclpy.spin_once(node,timeout_sec=.02)
  if last.get('metrics',{}).get('error'):raise RuntimeError(last['metrics']['error'])

def wait(p,seconds):
 end=time.monotonic()+seconds
 while p.poll() is None and time.monotonic()<end:spin(.05)
 if p.poll()!=0:raise RuntimeError('Command failed or timed out '+str(p.pid)+' '+str(p.poll()))

def ready(predicate,seconds=45):
 end=time.monotonic()+seconds
 while not predicate():
  if time.monotonic()>end:raise RuntimeError('Readiness timed out')
  spin(.1)

def service(typ,name,req):
 c=node.create_client(typ,'/ur7e_nvblox/'+name)
 if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Service missing '+name)
 f=c.call_async(req);end=time.monotonic()+35
 while not f.done() and time.monotonic()<end:spin(.05)
 if not f.done():raise RuntimeError('Service timeout '+name)
 res=f.result();node.destroy_client(c)
 if not res.success:raise RuntimeError(name+': '+res.message)
 return res

def param(name,value):
 p=start(['ros2','param','set','/ur7e_nvblox',name,str(value).lower() if isinstance(value,bool) else str(value)],'param_'+name+'_'+str(value));wait(p,10)
 # CLI exits 0 even when a parameter is rejected; inspect its actual output.
 text=(out/('param_'+name+'_'+str(value)+'.log')).read_text()
 if 'successful' not in text.lower():raise RuntimeError(text)

def capture(name):
 env=os.environ.copy()
 for k in ['LD_LIBRARY_PATH','PYTHONPATH','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH']:env.pop(k,None)
 x=subprocess.run(['/usr/bin/python3','-s',str(root/'tools/capture_rviz.py'),str(out/(name+'.png'))],env=env,capture_output=True,text=True,timeout=15)
 (out/(name+'_capture.txt')).write_text(x.stdout+x.stderr)
 if x.returncode:raise RuntimeError('Window capture failed '+x.stderr)

def hz(times):return (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None

def phase(name,seconds,motion=False):
 begin=time.monotonic();p=None
 if motion:p=start(['/usr/bin/python3',str(root/'tools/view_sweep.py'),'--seconds',str(seconds)],name+'_motion')
 spin(seconds*.6);capture(name);spin(seconds*.4)
 if p:wait(p,30)
 end=time.monotonic();ev=[x for x in clouds if begin<=x['wall']<=end];mm=[x for x in metrics if begin+2<=x[0]<=end]
 c=np.array([x['stamp'] for x in ev],np.int64);ts=[x['wall'] for x in ev];ages=[x['age_ms'] for x in ev if x['age_ms'] is not None]
 d={'duration_s':end-begin,'depth_input_hz':hz([t for t in depths if begin<=t<=end]),'rgb_input_hz':hz([t for t in rgbs if begin<=t<=end]),'mesh_hz':hz([t for t in meshes if begin<=t<=end]),'cloud_hz':hz(ts),'associated_new_input_hz':float((np.diff(c)>0).sum()/(ts[-1]-ts[0])),'repeated_headers':int((np.diff(c)==0).sum()),'points_min':min(x['points'] for x in ev),'points_max':max(x['points'] for x in ev),'age_p50_ms':float(np.percentile(ages,50)),'age_p95_ms':float(np.percentile(ages,95))}
 for key in ['depth_integrated','color_integrated','esdf_updated']:
  d[key+'_hz']=(mm[-1][1]['totals'].get(key,0)-mm[0][1]['totals'].get(key,0))/(mm[-1][0]-mm[0][0])
 r['phases'][name]=d;print('PHASE',name,json.dumps(d),flush=True)

def file_save(name,filename):
 req=FilePath.Request();req.file_path=str(out/filename);service(FilePath,name,req)
try:
 scene=start([str(root/'scene.sh'),'gazebo_gui:=false'],'scene',True)
 wait(start(['/usr/bin/python3',str(root/'tools/view_sweep.py'),'--check-only'],'scene_check'),45)
 wait(start(['/usr/bin/python3',str(root/'tools/view_sweep.py'),'--seconds','0'],'approach'),75)
 mapper=start([str(root/'run.sh'),'start_scene:=false','run_rviz:=false'],'mapper',True)
 ready(lambda:last.get('cloud') is not None and last['cloud'].width>0 and len(meshes)>5,60)
 rviz=start(['rviz2','-d',str(root/'install/ur7e_nvblox_in_hand/share/ur7e_nvblox_in_hand/rviz/mesh.rviz'),'--ros-args','-p','use_sim_time:=true'],'rviz_mesh',True);spin(4)
 assert any('rviz' in x.node_name.lower() for x in node.get_subscriptions_info_by_topic('/ur7e_nvblox/mesh')), 'RViz mesh subscription missing'
 assert (last['depth'].width,last['depth'].height)==(640,480) and (last['rgb'].width,last['rgb'].height)==(640,480)
 assert node.count_publishers('/ur7e_nvblox/esdf_points')==1 and node.count_publishers('/ur7e_nvblox/mesh')==1
 assert not any('workspaces/' in x for x in last['metrics']['loaded_libraries'])
 dump=subprocess.run(['ros2','param','dump','/ur7e_nvblox'],capture_output=True,text=True,timeout=10);(out/'parameters.yaml').write_text(dump.stdout)
 phase('static',12)
 phase('lateral_scan',20,motion=True)
 m=last['cloud'];a=np.frombuffer(m.data,dtype=np.dtype({'names':['x','y','z','d','observed','inside','site'],'formats':['<f4']*4+['u1']*3,'offsets':[0,4,8,12,20,21,22],'itemsize':24}))
 assert m.point_step==24 and len(a)>0 and np.all(a['observed']==1)
 xyz=np.stack([a['x'],a['y'],a['z']],axis=1);assert np.max(abs(xyz/.03-.5-np.round(xyz/.03-.5)))<1e-3
 np.save(out/'native_esdf.npy',a)
 req=QueryEsdf.Request();req.positions=xyz[:30].astype(np.float32).reshape(-1).tolist();ans=service(QueryEsdf,'get_esdf_and_gradient',req);assert all(ans.observed)
 req2=QueryEsdf.Request();req2.positions=[20.,20.,20.];ans2=service(QueryEsdf,'get_esdf_and_gradient',req2);assert not any(ans2.observed)
 r['checks']['native_esdf_and_query']={'passed':True,'points':len(a),'inside':int((a['inside']>0).sum()),'unknown_query':True}
 for key,file in [('save_ply','mesh.ply'),('save_map','map.blox'),('save_rates','rates.json'),('save_timings','timings.json')]:file_save(key,file)
 param('enable_color',False);spin(1);before=last['metrics']['totals'].get('color_integrated',0);spin(3);after=last['metrics']['totals'].get('color_integrated',0);assert after==before
 param('enable_color',True);spin(3);assert last['metrics']['totals']['color_integrated']>after;r['checks']['color_switch']=True
 critical.remove(rviz);stop(rviz)
 rviz=start(['rviz2','-d',str(root/'install/ur7e_nvblox_in_hand/share/ur7e_nvblox_in_hand/rviz/diagnostic.rviz'),'--ros-args','-p','use_sim_time:=true'],'rviz_esdf',True);spin(3);capture('esdf_observed')
 param('esdf_view_mode','inside');spin(2);inside=last['cloud'];flags=np.frombuffer(inside.data,np.uint8).reshape(-1,24)[:,21];assert len(flags)>0 and np.all(flags==1);r['checks']['inside_selection']=True
 param('esdf_view_mode','band');spin(2);band=np.frombuffer(last['cloud'].data,np.dtype({'names':['d'],'formats':['<f4'],'offsets':[12],'itemsize':24}));assert len(band)>0 and np.max(abs(band['d']))<=.24001;r['checks']['band_selection']=True
 param('enable_depth',False);param('enable_color',False);spin(1);service(Trigger,'clear_map',Trigger.Request());spin(1);assert last['cloud'].width==0;r['checks']['clear_map']=True
 param('enable_depth',True);param('enable_color',True);param('esdf_view_mode','observed');ready(lambda:last['cloud'].width>0,20)
 req=FilePath.Request();req.file_path=str(out/'map.blox');service(FilePath,'load_map',req);spin(2);r['checks']['load_map']=True
 r['last_metrics']=last['metrics']
 critical.remove(rviz);stop(rviz);critical.remove(mapper);stop(mapper)
 last['slice']=None;last['tsdf_layer']=None;last['color_layer']=None
 mapper=start([str(root/'run.sh'),'start_scene:=false','run_rviz:=false','esdf_mode:=2d'],'mapper_2d',True)
 ready(lambda:last.get('slice') is not None and last['slice'].width>0 and last.get('tsdf_layer') is not None and last['tsdf_layer'].width>0 and last.get('color_layer') is not None and last['color_layer'].width>0,60)
 rviz=start(['rviz2','-d',str(root/'install/ur7e_nvblox_in_hand/share/ur7e_nvblox_in_hand/rviz/2d.rviz'),'--ros-args','-p','use_sim_time:=true'],'rviz_2d',True);spin(3);capture('esdf_2d')
 slice_msg=last['slice'];z=np.frombuffer(slice_msg.data,np.dtype({'names':['z'],'formats':['<f4'],'offsets':[8],'itemsize':24}))['z'];assert len(np.unique(z))==1
 r['checks']['native_2d_slice']={'points':slice_msg.width,'z':float(z[0])}
 r['checks']['layers']={'tsdf_points':last['tsdf_layer'].width,'color_points':last['color_layer'].width}
 for key,counter in [('enable_layers','tsdf_layer_published'),('enable_mesh','mesh_published'),('publish_esdf_distance_slice','static_esdf_pointcloud_published')]:
  param(key,False);spin(1.5);n=last['metrics']['totals'].get(counter,0);spin(2);assert last['metrics']['totals'].get(counter,0)==n
  param(key,True);spin(2);assert last['metrics']['totals'].get(counter,0)>n;r['checks'][key+'_switch']=True
 r['last_metrics_2d']=last['metrics'];r['status']='completed'
except Exception as e:r['status']='failed';r['error']=str(e);traceback.print_exc()
finally:
 (out/'results.json').write_text(json.dumps(r,indent=2)+'\n');(out/'cloud_records.json').write_text(json.dumps(clouds));(out/'metrics.json').write_text(json.dumps(metrics));(out/'slice_records.json').write_text(json.dumps(slices))
 for p in reversed(procs):
  try:stop(p)
  except Exception:traceback.print_exc()
 for f in files:f.close()
 node.destroy_node();rclpy.try_shutdown()
 print('DONE',r['status'],str(out),flush=True)
if r['status']!='completed':raise SystemExit(1)
