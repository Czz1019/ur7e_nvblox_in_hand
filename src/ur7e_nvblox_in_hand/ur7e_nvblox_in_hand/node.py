"""ROS front end + single GPU owner + bounded, latest-output publication.

Map computation uses wheel C++/CUDA. Never loads nvblox_ros or Isaac ROS.
"""
from collections import deque,Counter,defaultdict
from dataclasses import dataclass,field
from pathlib import Path
import json,threading,time,queue,traceback
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor,ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from ur7e_nvblox_interfaces.srv import FilePath,QueryEsdf
from . import messages

DEFAULTS=dict(global_frame='base_link',voxel_size=.005,esdf_mode='3d',
    enable_depth=True,enable_color=True,enable_esdf=True,enable_mesh=True,enable_layers=True,
    enable_esdf_view=True,publish_pose=True,publish_path=True,publish_esdf_distance_slice=True,
    depth_hz=40.,color_hz=40.,esdf_hz=33.4,esdf_view_hz=33.4,mesh_hz=5.,layer_hz=5.,
    decay_hz=0.,clearing_hz=0.,clearing_radius_m=-1.,
    esdf_view_mode='observed',esdf_view_stride=2,esdf_view_band_m=.24,min_depth_m=.05,max_range_m=2.05,
    truncation_vox=4.,max_weight=5.,weighting_mode='inverse_square_tsdf_distance_penalty',
    invalid_depth_decay_factor=-1.,esdf_max_distance_m=2.,esdf_min_weight=.1,esdf_max_site_vox=2.,
    mesh_min_weight=.1,mesh_weld_vertices=True,layer_min_weight=.1,raycast_subsampling_factor=4,
    workspace_bounds_type='bounding_box',workspace_min_x=-.1,workspace_max_x=1.1,workspace_min_y=-.6,workspace_max_y=.6,workspace_min_z=-.05,workspace_max_z=1.05,
    tsdf_decay_factor=.95,tsdf_decayed_weight_threshold=.001,
    slice_min_height=0.,slice_max_height=.8,slice_height=.25,
    depth_topic='/camera/depth/image_rect_raw',depth_info_topic='/camera/depth/camera_info',
    color_topic='/camera/color/image_raw',color_info_topic='/camera/color/camera_info',
    input_qos='sensor_data',input_queue_size=4,max_input_age_sec=.5,path_max_poses=3000,max_query_points=200000,
    service_timeout_sec=20.)
MUTABLE={k for k in DEFAULTS if k.startswith('enable_') or k.endswith('_hz')}|{'publish_pose','publish_path','publish_esdf_distance_slice','esdf_view_mode','esdf_view_band_m','esdf_view_stride'}


def validate(cfg):
    if cfg['input_qos'] not in ('sensor_data','reliable'):raise ValueError('input_qos must be sensor_data or reliable')
    if cfg['esdf_mode'] not in ('2d','3d'):raise ValueError('esdf_mode must be 2d or 3d')
    if cfg['esdf_view_mode'] not in ('observed','band','inside'):raise ValueError('Invalid esdf_view_mode')
    for k in ['voxel_size','max_range_m','truncation_vox','max_weight','esdf_max_distance_m','esdf_view_band_m','max_input_age_sec','service_timeout_sec']:
        if not np.isfinite(cfg[k]) or cfg[k]<=0:raise ValueError(k+' must be finite and positive')
    for k in [x for x in cfg if x.endswith('_hz')]:
        if not np.isfinite(cfg[k]) or cfg[k]<0:raise ValueError(k+' must be finite and nonnegative')
    if cfg['min_depth_m']<0 or cfg['min_depth_m']>=cfg['max_range_m']:raise ValueError('Invalid depth range')
    if not 0<=cfg['tsdf_decay_factor']<=1:raise ValueError('Invalid decay factor')
    for k in ['esdf_min_weight','mesh_min_weight','layer_min_weight']:
        if not np.isfinite(cfg[k]) or cfg[k]<0:raise ValueError(k+' must be finite and nonnegative')
    for k in ['input_queue_size','path_max_poses','max_query_points','raycast_subsampling_factor']:
        if cfg[k]<1:raise ValueError(k+' must be positive')
    if isinstance(cfg['esdf_view_stride'],bool) or not isinstance(cfg['esdf_view_stride'],int) or cfg['esdf_view_stride']<1:raise ValueError('esdf_view_stride must be a positive integer')
    if cfg['slice_min_height']>cfg['slice_max_height']:raise ValueError('slice bounds reversed')
    if cfg['clearing_hz']>0 and cfg['clearing_radius_m']<=0:raise ValueError('clearing requires positive radius')
    if cfg['weighting_mode'] not in ('constant','constant_dropoff','inverse_square','inverse_square_dropoff','inverse_square_tsdf_distance_penalty'):raise ValueError('Unsupported weighting_mode for pinned wheel')
    if cfg['workspace_bounds_type'] not in ('unbounded','height_bounds','bounding_box'):raise ValueError('Invalid workspace_bounds_type')
    for axis in ['x','y','z']:
        if cfg['workspace_min_'+axis]>=cfg['workspace_max_'+axis]:raise ValueError('workspace bounds reversed')


@dataclass
class Job:
    kind:str
    request:object
    event:threading.Event=field(default_factory=threading.Event)
    result:object=None
    error:str=''
    deadline:float=0.
    cancelled:bool=False


class MapperNode(Node):
    def __init__(self):
        super().__init__('ur7e_nvblox')
        self.cfg={k:self.declare_parameter(k,v).value for k,v in DEFAULTS.items()};validate(self.cfg)
        self.lock=threading.RLock();self.cv=threading.Condition(self.lock)
        self.stop=threading.Event();self.wake_output=threading.Event();self.fatal=''
        self.inputs={k:deque(maxlen=self.cfg['input_queue_size']) for k in ('depth','color')}
        self.info={k:None for k in self.inputs};self.counts=Counter();self.times=defaultdict(lambda:deque(maxlen=200))
        self.outputs={};self.jobs=queue.Queue(maxsize=16);self.backend=None
        self.epoch=0;self.last_clock=0;self.last_seen={'depth':0,'color':0};self.latest_pose=None
        self.last_depth=0;self.esdf_stamp=0;self.esdf_version=0;self.exported_version=-1
        self.dirty_depth=False;self.dirty_mesh=False;self.last_fused={'depth':0,'color':0};self.deadlines={}
        self.path=deque(maxlen=self.cfg['path_max_poses']);self.last_path_wall=0.
        self.last_metrics_wall=time.monotonic();self.prev_counts={};self.latest_points=0
        self.buffer=Buffer(cache_time=Duration(seconds=15));self.listener=TransformListener(self.buffer,self)
        self.pubs={k:self.create_publisher(PointCloud2,'~/'+k,1) for k in ['esdf_points','static_esdf_pointcloud','tsdf_layer','color_layer']}
        self.pubs['mesh']=self.create_publisher(MarkerArray,'~/mesh',1)
        self.pose_pub=self.create_publisher(PoseStamped,'~/camera_pose',1)
        self.path_pub=self.create_publisher(RosPath,'~/camera_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.metrics_pub=self.create_publisher(String,'~/metrics',1)
        self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
        input_qos=(QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
                   if self.cfg['input_qos']=='reliable' else qos_profile_sensor_data)
        for kind in ['depth','color']:
            self.create_subscription(CameraInfo,self.cfg[kind+'_info_topic'],lambda m,k=kind:self.camera_info(k,m),input_qos)
            self.create_subscription(Image,self.cfg[kind+'_topic'],lambda m,k=kind:self.image(k,m),input_qos)
        self.service_handles=[]
        for kind in ['save_ply','save_map','load_map','save_rates','save_timings']:
            self.service_handles.append(self.create_service(FilePath,'~/'+kind,lambda req,res,k=kind:self.file_service(k,req,res)))
        self.service_handles.append(self.create_service(QueryEsdf,'~/get_esdf_and_gradient',self.query_service))
        self.service_handles.append(self.create_service(Trigger,'~/clear_map',self.clear_service))
        self.add_on_set_parameters_callback(self.parameters)
        self.create_timer(1.,self.metrics)
        self.worker=threading.Thread(target=self.work,name='nvblox-gpu-owner',daemon=True)
        self.publisher=threading.Thread(target=self.publish,name='nvblox-output',daemon=True)
        self.worker.start();self.publisher.start()

    def parameters(self,params):
        with self.lock:
            proposal=self.cfg.copy()
            for p in params:
                if p.name not in MUTABLE:return SetParametersResult(successful=False,reason=p.name+' requires restart/new map')
                proposal[p.name]=p.value
            try:validate(proposal)
            except ValueError as e:return SetParametersResult(successful=False,reason=str(e))
            self.cfg.update(proposal);self.cv.notify_all()
        return SetParametersResult(successful=True)

    def clock(self,m):
        now=m.clock.sec*10**9+m.clock.nanosec
        with self.cv:
            if self.last_clock and now+1000000<self.last_clock:
                for q in self.inputs.values():q.clear()
                self.last_seen={'depth':0,'color':0};self.counts['clock_resets']+=1
                try:self.jobs.put_nowait(Job('clear_map',None))
                except queue.Full:self.fatal='clock reset queue full'
            self.last_clock=now;self.cv.notify_all()

    def camera_info(self,kind,m):
        if m.k[0]<=0 or m.k[4]<=0:return
        with self.lock:self.info[kind]=m

    def image(self,kind,m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        with self.cv:
            self.counts[kind+'_received']+=1
            if not self.cfg['enable_'+kind]:return
            if stamp<=0 or stamp<=self.last_seen[kind]:self.counts[kind+'_invalid_stamp']+=1;return
            self.last_seen[kind]=stamp
            if len(self.inputs[kind])==self.inputs[kind].maxlen:self.counts[kind+'_overwritten']+=1
            self.inputs[kind].append((m,stamp,time.monotonic()));self.cv.notify_all()

    def candidate(self,kind):
        with self.lock:
            if not self.inputs[kind] or self.info[kind] is None:return None
            m,stamp,received=self.inputs[kind][0];info=self.info[kind]
            if (m.width,m.height,m.header.frame_id)!=(info.width,info.height,info.header.frame_id):
                self.inputs[kind].popleft();self.counts[kind+'_camera_info_mismatch']+=1;return None
        now=time.monotonic();rosnow=self.get_clock().now().nanoseconds
        if now-received>self.cfg['max_input_age_sec'] or (rosnow>stamp and (rosnow-stamp)*1e-9>self.cfg['max_input_age_sec']):
            with self.lock:
                if self.inputs[kind] and self.inputs[kind][0][1]==stamp:self.inputs[kind].popleft();self.counts[kind+'_stale']+=1
            return None
        if stamp>rosnow+100000000:return None
        hz=self.cfg[kind+'_hz']
        if hz<=0 or stamp-self.last_fused[kind]<round(1e9/hz):
            with self.lock:
                if self.inputs[kind] and self.inputs[kind][0][1]==stamp:self.inputs[kind].popleft();self.counts[kind+'_gate_skipped']+=1
            return None
        try:tf=self.buffer.lookup_transform(self.cfg['global_frame'],m.header.frame_id,Time.from_msg(m.header.stamp))
        except TransformException:return None # retry until bounded age; never substitute latest TF.
        q=tf.transform.rotation;t=tf.transform.translation
        pose=np.eye(4,dtype=np.float32);pose[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();pose[:3,3]=[t.x,t.y,t.z]
        with self.lock:
            if not self.inputs[kind] or self.inputs[kind][0][1]!=stamp:return None
            self.inputs[kind].popleft()
        return m,stamp,info,pose,tf

    def due(self,name,hz):
        if hz<=0:return False
        now=time.monotonic()
        if now<self.deadlines.get(name,0):return False
        self.deadlines[name]=now+1./hz;return True

    def enqueue(self,name,payload):
        with self.lock:
            if name in self.outputs:self.counts[name+'_output_overwritten']+=1
            self.outputs[name]=(self.epoch,payload)
        self.wake_output.set()

    def update_esdf(self):
        start=time.monotonic();self.backend.update_esdf()
        with self.lock:self.counts['esdf_updated']+=1;self.esdf_version+=1;self.esdf_stamp=self.last_depth
        self.dirty_depth=False;self.record_time('esdf_submit_ms',(time.monotonic()-start)*1000)

    def pose(self,stamp,tf):
        m=PoseStamped();m.header=messages.header(self.cfg['global_frame'],stamp)
        m.pose.position.x=tf.transform.translation.x;m.pose.position.y=tf.transform.translation.y;m.pose.position.z=tf.transform.translation.z;m.pose.orientation=tf.transform.rotation
        if self.cfg['publish_pose']:self.enqueue('camera_pose',m)
        if self.cfg['publish_path'] and time.monotonic()-self.last_path_wall>.2:
            self.path.append(m);p=RosPath();p.header=m.header;p.poses=list(self.path);self.enqueue('camera_path',p);self.last_path_wall=time.monotonic()

    def work(self):
        try:
            from .backend import Backend
            self.backend=Backend(self.cfg)
            self.get_logger().info('Wheel C++/CUDA Mapper ready; native ESDF bridge loaded; no Isaac ROS.')
            while not self.stop.is_set():
                try:
                    job=self.jobs.get_nowait()
                    if not job.cancelled and (not job.deadline or time.monotonic()<job.deadline):
                        try:job.result=self.execute(job)
                        except Exception as e:job.error=str(e)
                    else:job.error='Request expired before execution'
                    job.event.set()
                except queue.Empty:pass
                for kind in ['depth','color']:
                    if not self.cfg['enable_'+kind]:continue
                    item=self.candidate(kind)
                    if item is None:continue
                    m,stamp,info,pose,tf=item
                    if kind=='color' and not self.last_depth:
                        with self.lock:self.counts['color_before_geometry']+=1
                        continue
                    begin=time.monotonic()
                    try:getattr(self.backend,kind)(m,pose,info)
                    except ValueError as e:
                        with self.lock:self.counts[kind+'_invalid_image']+=1
                        self.get_logger().warning(str(e));continue
                    self.last_fused[kind]=stamp
                    with self.lock:self.counts[kind+'_integrated']+=1
                    self.record_time(kind+'_submit_ms',(time.monotonic()-begin)*1000)
                    self.dirty_mesh=True
                    if kind=='depth':
                        self.last_depth=stamp;self.latest_pose=pose;self.dirty_depth=True;self.pose(stamp,tf)
                if self.cfg['enable_esdf'] and self.dirty_depth and self.due('esdf',self.cfg['esdf_hz']):self.update_esdf()
                # Equal/faster display follows an already rate-limited ESDF version; avoid two independent rejection gates.
                view_due=(self.cfg['esdf_view_hz']>0 and ((0<self.cfg['esdf_hz']<=self.cfg['esdf_view_hz']) or self.due('view',self.cfg['esdf_view_hz'])))
                slice_enabled=(self.cfg['esdf_mode']=='3d' or self.cfg['publish_esdf_distance_slice'])
                if self.cfg['enable_esdf'] and self.cfg['enable_esdf_view'] and slice_enabled and self.esdf_version!=self.exported_version and view_due:
                    name='esdf_points' if self.cfg['esdf_mode']=='3d' else 'static_esdf_pointcloud'
                    if self.pubs[name].get_subscription_count():
                        begin=time.monotonic();data=self.backend.esdf(self.cfg['esdf_view_mode'],self.cfg['esdf_view_band_m'],self.cfg['esdf_view_stride'] if self.cfg['esdf_mode']=='3d' else 1)
                        self.enqueue(name,(data,self.esdf_stamp,self.esdf_version));self.exported_version=self.esdf_version
                        self.record_time('esdf_export_with_gpu_wait_ms',(time.monotonic()-begin)*1000)
                        self.latest_points=len(data)
                if self.cfg['enable_mesh'] and self.dirty_mesh and self.pubs['mesh'].get_subscription_count() and self.due('mesh',self.cfg['mesh_hz']):
                    begin=time.monotonic();mesh=self.backend.mesh();self.enqueue('mesh',(mesh,self.last_depth,self.esdf_version));self.dirty_mesh=False
                    self.record_time('mesh_readback_ms',(time.monotonic()-begin)*1000)
                    with self.lock:self.counts['mesh_generated']+=1
                if self.cfg['enable_layers'] and self.last_depth and self.due('layers',self.cfg['layer_hz']):
                    for kind,color in [('tsdf_layer',False),('color_layer',True)]:
                        if self.pubs[kind].get_subscription_count():self.enqueue(kind,(self.backend.layer(color),self.last_depth,0))
                if self.last_depth and self.due('decay',self.cfg['decay_hz']):
                    self.backend.mapper.decay(0);self.dirty_depth=self.dirty_mesh=True
                    with self.lock:self.counts['decay_calls']+=1
                if self.latest_pose is not None and self.due('clearing',self.cfg['clearing_hz']):
                    self.backend.clear_radius(self.latest_pose[:3,3]);self.dirty_depth=self.dirty_mesh=True
                    with self.lock:self.counts['clearing_calls']+=1
                with self.cv:self.cv.wait(timeout=.002)
        except Exception as e:
            self.fatal=str(e);self.get_logger().error(traceback.format_exc());self.stop.set();self.wake_output.set()
        finally:
            while True:
                try:job=self.jobs.get_nowait();job.error='Mapper stopped';job.event.set()
                except queue.Empty:break

    def publish(self):
        try:
            while not self.stop.is_set():
                self.wake_output.wait(.1);self.wake_output.clear()
                with self.lock:items=self.outputs;self.outputs={}
                for name,(epoch,payload) in items.items():
                    if epoch!=self.epoch:continue
                    if name=='camera_pose':self.pose_pub.publish(payload);continue
                    if name=='camera_path':self.path_pub.publish(payload);continue
                    data,stamp,version=payload
                    begin=time.monotonic();frame=self.cfg['global_frame']
                    if name=='mesh':msg=messages.mesh_serialized(data,frame,stamp)
                    elif name in ('esdf_points','static_esdf_pointcloud'):msg=messages.esdf_cloud(data,frame,stamp)
                    else:msg=messages.layer_cloud(data,frame,stamp,name=='color_layer')
                    if epoch!=self.epoch:continue
                    self.pubs[name].publish(msg)
                    with self.lock:self.counts[name+'_published']+=1
                    self.record_time(name+'_pack_publish_ms',(time.monotonic()-begin)*1000)
        except Exception as e:self.fatal='Publisher: '+str(e);self.get_logger().error(traceback.format_exc());self.stop.set()

    def request(self,kind,request):
        if self.stop.is_set():raise RuntimeError(self.fatal or 'Mapper stopped')
        job=Job(kind,request,deadline=time.monotonic()+self.cfg['service_timeout_sec'])
        self.jobs.put_nowait(job)
        with self.cv:self.cv.notify_all()
        if not job.event.wait(self.cfg['service_timeout_sec']):job.cancelled=True;raise RuntimeError('Service timed out; running GPU operation may still complete')
        if job.error:raise RuntimeError(job.error)
        return job.result

    def file_service(self,kind,req,res):
        try:res.message=str(self.request(kind,req.file_path));res.success=True
        except Exception as e:res.message=str(e);res.success=False
        return res

    def query_service(self,req,res):
        try:
            d,g,observed,stamp=self.request('query',req)
            res.distances=d.tolist();res.gradients=g.reshape(-1).tolist();res.observed=observed.tolist();res.input_stamp_ns=stamp;res.voxel_size=float(self.cfg['voxel_size']);res.frame_id=self.cfg['global_frame'];res.success=True
        except Exception as e:res.success=False;res.message=str(e)
        return res

    def clear_service(self,req,res):
        try:self.request('clear_map',None);res.success=True;res.message='Map, queued input and visible outputs reset'
        except Exception as e:res.success=False;res.message=str(e)
        return res

    def execute(self,job):
        if job.kind=='clear_map':
            self.backend.mapper.clear(0)
            with self.lock:
                for q in self.inputs.values():q.clear()
                self.epoch+=1;self.last_fused={'depth':0,'color':0};self.last_depth=0;self.esdf_stamp=0;self.last_seen={'depth':0,'color':0};self.latest_pose=None
                self.dirty_depth=self.dirty_mesh=False;self.esdf_version=0;self.exported_version=-1;self.path.clear();self.outputs.clear();self.latest_points=0
            empty=np.empty((0,7),np.float32)
            for name in ['esdf_points','static_esdf_pointcloud']:self.enqueue(name,(empty,0,0))
            self.enqueue('mesh',((np.empty((0,3)),np.empty((0,3),np.int32),np.empty((0,3))),0,0))
            for name in ['tsdf_layer','color_layer']:self.enqueue(name,(np.empty((0,8)),0,0))
            path=RosPath();path.header=messages.header(self.cfg['global_frame'],0);self.enqueue('camera_path',path)
            return 'cleared'
        if job.kind=='query':
            req=job.request
            if req.update_esdf:self.update_esdf()
            if req.use_aabb:
                sizes=np.asarray(req.aabb_size);origin=np.asarray(req.aabb_min);v=self.cfg['voxel_size']
                if not np.isfinite(sizes).all() or not np.isfinite(origin).all() or np.any(sizes<=0):raise ValueError('Invalid AABB')
                import math
                shape=tuple(math.ceil(float(x)/v) for x in sizes)
                if math.prod(shape)>self.cfg['max_query_points']:raise ValueError('AABB exceeds max_query_points')
                axes=[origin[i]+(np.arange(shape[i])+.5)*v for i in range(3)];points=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
            else:
                if len(req.positions)%3:raise ValueError('positions must be XYZ triples')
                points=np.asarray(req.positions,np.float32).reshape(-1,3)
            if len(points)<1 or len(points)>self.cfg['max_query_points'] or not np.isfinite(points).all():raise ValueError('Invalid query size or coordinates')
            return (*self.backend.query(points),self.esdf_stamp)
        path=Path(job.request).expanduser()
        if not str(job.request).strip() or not path.is_absolute():raise ValueError('file_path must be absolute and nonempty')
        path.parent.mkdir(parents=True,exist_ok=True)
        if job.kind=='save_map':self.backend.mapper.save_map(str(path),0)
        elif job.kind=='load_map':
            if not path.is_file():raise ValueError('Map file missing')
            from .backend import Backend
            replacement=Backend(self.cfg)
            replacement.mapper.load_from_file(str(path),0)
            actual=replacement.ops.voxel_size(replacement.mapper.get_c_mapper())
            if abs(actual-self.cfg['voxel_size'])>1e-6:raise ValueError('Map voxel size differs from configured voxel_size; old map retained')
            self.backend=replacement
            with self.lock:
                self.epoch+=1;self.last_depth=self.esdf_stamp=0;self.dirty_depth=self.dirty_mesh=True
                self.outputs.clear();self.path.clear();self.last_fused={'depth':0,'color':0}
                for q in self.inputs.values():q.clear()
        elif job.kind=='save_ply':
            self.backend.mapper.update_color_mesh(0);self.backend.mapper.get_color_mesh(0).save(str(path))
        elif job.kind in ('save_rates','save_timings'):
            path.write_text(json.dumps(self.snapshot(),indent=2)+'\n')
        if not path.is_file():raise RuntimeError('Backend did not create requested file')
        return str(path)

    def record_time(self,key,value):
        with self.lock:self.times[key].append(value)

    def snapshot(self):
        with self.lock:
            return {'backend':'nvblox_torch_cpp_cuda','map_epoch':self.epoch,'totals':dict(self.counts),'native_esdf_points':self.latest_points,
                'esdf_exported_points':self.latest_points,'esdf_view_stride':self.cfg['esdf_view_stride'],
                'esdf_view_spacing_m':self.cfg['voxel_size']*(self.cfg['esdf_view_stride'] if self.cfg['esdf_mode']=='3d' else 1),
                'nvblox_core_library':str(self.backend.library) if self.backend is not None else None,'loaded_libraries':self.backend.loaded_libraries if self.backend is not None else [],
                'input_stamp_ns':self.last_depth,'esdf_input_stamp_ns':self.esdf_stamp,'voxel_size_m':self.cfg['voxel_size'],
                'esdf_version':self.esdf_version,'gpu_completion_measured':False,'error':self.fatal,
                'timings_host_ms':{k:{'mean':float(np.mean(v)),'p95':float(np.percentile(v,95))} for k,v in self.times.items() if v},
                'pose_source':'robot_tf_at_each_image_stamp','esdf_export':'native_observed_voxels','configuration':self.cfg.copy()}

    def metrics(self):
        d=self.snapshot();now=time.monotonic();dt=now-self.last_metrics_wall
        d['wall_hz']={k:(v-self.prev_counts.get(k,0))/dt for k,v in d['totals'].items()}
        self.prev_counts=d['totals'];self.last_metrics_wall=now
        msg=String();msg.data=json.dumps(d);self.metrics_pub.publish(msg)
        if self.fatal:raise RuntimeError(self.fatal)

    def close(self):
        self.stop.set();self.wake_output.set()
        with self.cv:self.cv.notify_all()
        self.worker.join(timeout=15);self.publisher.join(timeout=5)


def main(args=None):
    rclpy.init(args=args);node=MapperNode();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();executor.shutdown();node.destroy_node();rclpy.try_shutdown()
