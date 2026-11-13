"""Single-owner GPU backend. No ROS / Isaac ROS / sibling-workspace imports."""
import ctypes
import importlib.util
import os
from pathlib import Path
import runpy
import numpy as np


class Backend:
    def __init__(self, cfg):
        import torch
        self.torch=torch;self.cfg=cfg
        spec=importlib.util.find_spec('nvblox_torch')
        if spec is None:raise RuntimeError('nvblox_torch missing; run install_dependencies.sh')
        package=Path(spec.origin).parent
        version=runpy.run_path(str(package/'_version.py'))
        if version.get('__git_sha__')!='e4a2b00f45ecf989682d72a33a2c29defe39fd2d':
            raise RuntimeError('Wheel commit differs from vendored native bridge headers')
        self.library=package/'lib/nvblox/libnvblox_lib.so'
        ctypes.CDLL(str(self.library),mode=ctypes.RTLD_GLOBAL)
        from nvblox_torch.mapper import Mapper,QueryType
        from nvblox_torch.mapper_params import MapperParams
        from nvblox_torch.constants import constants
        from nvblox_torch.sensor import Sensor
        self.QueryType=QueryType;self.Sensor=Sensor
        self.unknown=float(constants.esdf_unknown_distance())
        params=MapperParams()
        groups={
            'projective_integrator':{
                'projective_integrator_max_integration_distance_m':cfg['max_range_m'],
                'projective_integrator_truncation_distance_vox':cfg['truncation_vox'],
                'projective_integrator_weighting_mode':{'constant':'kConstantWeight','constant_dropoff':'kConstantDropoffWeight','inverse_square':'kInverseSquareWeight','inverse_square_dropoff':'kInverseSquareDropoffWeight','inverse_square_tsdf_distance_penalty':'kInverseSquareTsdfDistancePenalty'}[cfg['weighting_mode']],
                'projective_integrator_max_weight':cfg['max_weight'],
                'projective_tsdf_integrator_invalid_depth_decay_factor':cfg['invalid_depth_decay_factor']},
            'esdf_integrator':{
                'esdf_integrator_max_distance_m':cfg['esdf_max_distance_m'],
                'unobserved_esdf_policy':'kIgnore',
                'esdf_integrator_min_weight':cfg['esdf_min_weight'],
                'esdf_integrator_max_site_distance_vox':cfg['esdf_max_site_vox'],
                'esdf_slice_min_height':cfg['slice_min_height'],
                'esdf_slice_max_height':cfg['slice_max_height'],
                'esdf_slice_height':cfg['slice_height']},
            'mesh_integrator':{'mesh_integrator_min_weight':cfg['mesh_min_weight'],
                               'mesh_integrator_weld_vertices':cfg['mesh_weld_vertices']},
            'view_calculator':{'raycast_subsampling_factor':cfg['raycast_subsampling_factor'],
                'workspace_bounds_type':{'unbounded':'kUnbounded','height_bounds':'kHeightBounds','bounding_box':'kBoundingBox'}[cfg['workspace_bounds_type']],
                'workspace_bounds_min_height_m':cfg['workspace_min_z'],
                'workspace_bounds_max_height_m':cfg['workspace_max_z'],
                'workspace_bounds_min_corner_x_m':cfg['workspace_min_x'],
                'workspace_bounds_max_corner_x_m':cfg['workspace_max_x'],
                'workspace_bounds_min_corner_y_m':cfg['workspace_min_y'],
                'workspace_bounds_max_corner_y_m':cfg['workspace_max_y']},
            'tsdf_decay_integrator':{'tsdf_decay_factor':cfg['tsdf_decay_factor'],
                'tsdf_decayed_weight_threshold':cfg['tsdf_decayed_weight_threshold'],
                'tsdf_set_free_distance_on_decayed':False}}
        for group,values in groups.items():
            sub=getattr(params,'get_'+group+'_params')()
            for name,value in values.items():getattr(sub._c_params,'set_'+name)(value)
            getattr(params,'set_'+group+'_params')(sub)
        self.mapper=Mapper(float(cfg['voxel_size']),mapper_parameters=params)
        bridge=Path(os.environ.get('UR7E_NVBLOX_BRIDGE',''))
        if not bridge.is_file():raise RuntimeError('Native export bridge missing; run ./build.sh')
        torch.ops.load_library(str(bridge))
        self.ops=torch.ops.ur7e_nvblox_native
        self.loaded_libraries=sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines() if '/' in line and any(x in line for x in ('libnvblox','libpy_nvblox','libcudart','libtorch_cuda'))})
        self.sensors={}

    def sensor(self, info):
        # Gazebo fills K; real rectified input may use P. Use the supplied camera model.
        k=info.k;key=(info.width,info.height,tuple(k))
        if key not in self.sensors:
            self.sensors[key]=self.Sensor.from_camera(float(k[0]),float(k[4]),float(k[2]),float(k[5]),int(info.width),int(info.height))
        return self.sensors[key]

    def depth(self, image, pose, info):
        dtype=np.dtype(('>' if image.is_bigendian else '<')+('u2' if image.encoding=='16UC1' else 'f4'))
        if image.encoding not in ('16UC1','32FC1'):raise ValueError('Depth encoding must be 16UC1 or 32FC1')
        if image.step<image.width*dtype.itemsize or len(image.data)<image.step*image.height:raise ValueError('Truncated depth buffer')
        a=np.ndarray((image.height,image.width),dtype=dtype,buffer=image.data,strides=(image.step,dtype.itemsize)).astype(np.float32,copy=True)
        if image.encoding=='16UC1':a*=.001
        a[~np.isfinite(a)|(a<self.cfg['min_depth_m'])|(a>self.cfg['max_range_m'])]=0
        gpu=self.torch.as_tensor(a,device='cuda',dtype=self.torch.float32)
        self.mapper.add_depth_frame(gpu,self.torch.from_numpy(pose),self.sensor(info))

    def color(self, image, pose, info):
        if image.encoding not in ('rgb8','bgr8'):raise ValueError('Color encoding must be rgb8 or bgr8')
        if image.step<image.width*3 or len(image.data)<image.step*image.height:raise ValueError('Truncated RGB buffer')
        a=np.ndarray((image.height,image.width,3),np.uint8,buffer=image.data,strides=(image.step,3,1))
        if image.encoding=='bgr8':a=a[:,:,::-1]
        gpu=self.torch.as_tensor(np.ascontiguousarray(a),device='cuda',dtype=self.torch.uint8)
        self.mapper.add_color_frame(gpu,self.torch.from_numpy(pose),self.sensor(info))

    def update_esdf(self):
        if self.cfg['esdf_mode']=='3d':self.mapper.update_esdf(0)
        else:self.ops.update_esdf_slice(self.mapper.get_c_mapper())

    def esdf(self, selection, band, stride=1):
        kind={'observed':0,'band':1,'inside':2}[selection]
        if isinstance(stride,bool) or not isinstance(stride,int) or stride<1:raise ValueError('ESDF stride must be a positive integer')
        if stride==1:return self.ops.export_esdf(self.mapper.get_c_mapper(),kind,float(band)).numpy().copy()
        return self.ops.export_esdf_strided(self.mapper.get_c_mapper(),kind,float(band),stride).numpy().copy()

    def layer(self, color=False):
        return self.ops.export_layer(self.mapper.get_c_mapper(),int(color),float(self.cfg['layer_min_weight'])).numpy().copy()

    def mesh(self):
        self.mapper.update_color_mesh(0);m=self.mapper.get_color_mesh(0)
        return tuple(x.detach().cpu().numpy().copy() for x in (m.vertices(),m.triangles(),m.vertex_colors()))

    def query(self, points):
        points=np.asarray(points,np.float32).reshape(-1,3)
        q=self.torch.zeros((len(points),4),device='cuda',dtype=self.torch.float32)
        q[:,:3]=self.torch.as_tensor(points,device='cuda')
        out=self.mapper.query_layer(self.QueryType.ESDF_GRAD,q,mapper_id=0).detach().cpu().numpy().copy()
        # Binding API: [gradient x,y,z,distance]. Unknown stays explicit, never "free".
        valid=np.isfinite(out).all(1)&(np.abs(out[:,3])<abs(self.unknown)*.99)
        return out[:,3],out[:,:3],valid

    def clear_radius(self, center):
        self.ops.clear_radius(self.mapper.get_c_mapper(),*[float(x) for x in center],float(self.cfg['clearing_radius_m']))
