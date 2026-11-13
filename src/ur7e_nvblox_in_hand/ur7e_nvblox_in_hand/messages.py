"""ROS packing on the publication thread; all inputs own their CPU memory."""
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header,ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray


def header(frame,stamp):
    h=Header();h.frame_id=frame;h.stamp.sec=int(stamp//10**9);h.stamp.nanosec=int(stamp%10**9);return h


def esdf_cloud(a,frame,stamp):
    dt=np.dtype({'names':['x','y','z','intensity','rgb','observed','is_inside','is_site','pad'],
       'formats':['<f4']*4+['<u4']+['u1']*4,'offsets':[0,4,8,12,16,20,21,22,23],'itemsize':24})
    out=np.zeros(len(a),dt)
    for i,k in enumerate(['x','y','z','intensity']):out[k]=a[:,i]
    for i,k in enumerate(['observed','is_inside','is_site'],4):out[k]=a[:,i].astype(np.uint8)
    t=np.minimum(1,np.abs(a[:,3])/.24);out['rgb']=np.where(a[:,5]>0,0xff3030,((255*(1-t)).astype(np.uint32)<<8)|(255*t).astype(np.uint32))
    msg=PointCloud2();msg.header=header(frame,stamp);msg.height=1;msg.width=len(a);msg.point_step=24;msg.row_step=len(out)*24;msg.is_dense=True
    msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z','intensity'])]
    msg.fields.append(PointField(name='rgb',offset=16,datatype=PointField.UINT32,count=1))
    msg.fields.extend(PointField(name=k,offset=i,datatype=PointField.UINT8,count=1) for i,k in enumerate(['observed','is_inside','is_site'],20));msg.data=array('B',out.tobytes());return msg


def layer_cloud(a,frame,stamp,color):
    out=np.zeros(len(a),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('rgb','<u4'),('weight','<f4')])
    for i,k in enumerate(['x','y','z']):out[k]=a[:,i]
    out['intensity']=a[:,3];out['weight']=a[:,4]
    if color:out['rgb']=(a[:,5].astype(np.uint32)<<16)|(a[:,6].astype(np.uint32)<<8)|a[:,7].astype(np.uint32)
    msg=PointCloud2();msg.header=header(frame,stamp);msg.height=1;msg.width=len(a);msg.point_step=24;msg.row_step=len(a)*24;msg.is_dense=True
    msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.UINT32 if k=='rgb' else PointField.FLOAT32,count=1) for i,k in enumerate(out.dtype.names)];msg.data=array('B',out.tobytes());return msg


def mesh_markers(data,frame,stamp):
    vertices,triangles,colors=data
    msg=MarkerArray();m=Marker();m.header=header(frame,stamp);m.ns='nvblox_mesh';m.id=0;m.type=Marker.TRIANGLE_LIST;m.action=Marker.ADD
    m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=1.;m.color.a=1.
    if not len(triangles):m.action=Marker.DELETE
    if len(triangles):
        ids=triangles.astype(np.int64).reshape(-1);xyz=vertices[ids]
        rgb=colors[ids].astype(np.float32)/255.
        m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in xyz]
        m.colors=[ColorRGBA(r=float(x),g=float(y),b=float(z),a=1.) for x,y,z in rgb]
    msg.markers=[m];return msg

# Build an ABI/schema-aware template through the installed ROS type support.
# Bulk-fill its Point/Color sequences; never hard-code the distribution's trailing fields.
from functools import lru_cache
import struct
from array import array
from rclpy.serialization import serialize_message


@lru_cache(maxsize=8)
def _mesh_template(frame):
    v=np.array([[1234567.25,-54321.5,98765.125]],np.float64)
    marker=Marker();marker.header=header(frame,0x12345678*10**9+0x23456789)
    marker.ns='nvblox_mesh';marker.id=0;marker.type=Marker.TRIANGLE_LIST;marker.action=Marker.ADD
    marker.pose.orientation.w=1.;marker.scale.x=marker.scale.y=marker.scale.z=1.;marker.color.a=1.
    empty=serialize_message(MarkerArray(markers=[marker]))
    marker.points=[Point(x=float(v[0,0]),y=float(v[0,1]),z=float(v[0,2]))]
    marker.colors=[ColorRGBA(r=.125,g=.25,b=.5,a=1.)]
    one=serialize_message(MarkerArray(markers=[marker]))
    count=next(i for i,(a,b) in enumerate(zip(empty,one)) if a!=b)
    start=one.index(v.astype('<f8').tobytes())
    assert struct.unpack_from('<I',one,count)[0]==1 and (start-4)%8==0
    assert start-count-4 in (0,4)
    color_count=start+24;assert struct.unpack_from('<I',one,color_count)[0]==1
    color_start=color_count+4
    assert one[color_start:color_start+16]==struct.pack('<ffff',.125,.25,.5,1.)
    prefix=one[:count];stamp_offset=prefix.index(struct.pack('<iI',0x12345678,0x23456789))
    return prefix,start-count-4,one[color_start+16:],stamp_offset


def mesh_serialized(data,frame,stamp):
    """Standard MarkerArray CDR, vectorized; validated against ROS serializer tests."""
    vertices,triangles,colors=data
    if not len(triangles):return serialize_message(mesh_markers(data,frame,stamp))
    ids=triangles.astype(np.int64,copy=False).reshape(-1)
    xyz=np.ascontiguousarray(vertices[ids],dtype='<f8')
    rgba=np.ones((len(ids),4),dtype='<f4');rgba[:,:3]=colors[ids].astype(np.float32)/255.
    prefix,padding,suffix,offset=_mesh_template(frame)
    prefix=bytearray(prefix);struct.pack_into('<iI',prefix,offset,int(stamp//10**9),int(stamp%10**9))
    return b''.join([prefix,struct.pack('<I',len(ids)),bytes(padding),xyz.tobytes(),struct.pack('<I',len(ids)),rgba.tobytes(),suffix])
