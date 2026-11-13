#!/usr/bin/env python3
"""Meaningful GPU contract test: native flags, centres, queries, mesh, persistence and 2D."""
import sys,json,tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np,yaml
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src/ur7e_nvblox_in_hand'))
from ur7e_nvblox_in_hand.backend import Backend
cfg=yaml.safe_load((root/'src/ur7e_nvblox_in_hand/config/base.yaml').read_text())['/**']['ros__parameters']
info=SimpleNamespace(width=64,height=48,k=[60.,0.,32.,0.,60.,24.,0.,0.,1.])
depth=np.full((48,64),1000,np.uint16);rgb=np.zeros((48,64,3),np.uint8);rgb[:24,:,0]=220;rgb[24:,:,1]=220
image=SimpleNamespace(width=64,height=48,encoding='16UC1',is_bigendian=False,step=128,data=depth.tobytes())
color=SimpleNamespace(width=64,height=48,encoding='rgb8',step=192,data=rgb.tobytes())
pose=np.eye(4,dtype=np.float32)
b=Backend(cfg)
for _ in range(4):b.depth(image,pose,info);b.color(color,pose,info)
b.update_esdf();a=b.esdf('observed',.24)
assert len(a)>0 and np.isfinite(a).all() and np.all(a[:,4]==1)
assert np.max(abs(a[:,:3]/cfg['voxel_size']-.5-np.round(a[:,:3]/cfg['voxel_size']-.5)))<1e-3
assert (a[:,5]>0).any() and (a[:,5]==0).any()
assert (a[a[:,5]>0,3]<=0).all() and (a[a[:,5]==0,3]>=0).all()
ids=np.unique(np.r_[np.linspace(0,len(a)-1,300).astype(int),np.where(a[:,5]>0)[0][:20]])
d,g,known=b.query(a[ids,:3]);assert known.all();err=float(np.max(abs(d-a[ids,3])))
assert err<cfg['voxel_size']*1.1,('query/native distance mismatch',err,d[:5],a[ids[:5],3])
d0,g0,k0=b.query([[20,20,20]]);assert not k0.any()
mesh=b.mesh();assert len(mesh[0]) and len(mesh[1]);assert len(mesh[2])==len(mesh[0])
assert len(b.layer(False)) and len(b.layer(True))
with tempfile.TemporaryDirectory() as directory:
 file=Path(directory)/'plane.blox';b.mapper.save_map(str(file),0);assert file.is_file()
 b.mapper.clear(0);assert len(b.esdf('observed',.24))==0
 b.mapper.load_from_file(str(file),0);b.update_esdf();assert len(b.esdf('observed',.24))>0
b.cfg['clearing_radius_m']=.3;b.clear_radius([0,0,0]);b.mapper.clear(0)
cfg=cfg.copy();cfg['esdf_mode']='2d';b2=Backend(cfg)
pose[:3,:3]=[[1,0,0],[0,0,1],[0,-1,0]];pose[2,3]=.5
for _ in range(3):b2.depth(image,pose,info)
b2.update_esdf();slice=b2.esdf('observed',.24);assert len(slice)>0
assert len(np.unique(np.round(slice[:,2],5)))==1
summary={'status':'passed','native_esdf_points':len(a),'inside':int((a[:,5]>0).sum()),'mesh_vertices':len(mesh[0]),'mesh_triangles':len(mesh[1]),'query_native_max_abs_error_m':err,'unknown_query_recognized':True,'save_load_clear':True,'slice_points':len(slice),'slice_z':float(slice[0,2])}
print(json.dumps(summary,indent=2))
(root/'artifacts').mkdir(exist_ok=True);(root/'artifacts/backend_contract_test.json').write_text(json.dumps(summary,indent=2)+'\n')
