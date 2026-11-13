"""Procedural static laboratory bench. Geometry and collision share SI dimensions."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

CAPS = [(0.85, .13, .12, 1), (.10, .48, .85, 1), (.96, .70, .08, 1),
        (.14, .65, .32, 1), (.56, .25, .68, 1)]


def annulus_stl(path, inner, outer, height, segments=32):
    """Closed ring with a genuinely open centre; no painted-on hole."""
    vertices = []
    for z in (-height/2, height/2):
        for r in (inner, outer):
            vertices.extend((r*math.cos(2*math.pi*i/segments), r*math.sin(2*math.pi*i/segments), z)
                            for i in range(segments))
    faces = []
    for i in range(segments):
        j = (i+1) % segments
        a,b,c,d=i,j,segments+j,segments+i
        A,B,C,D=a+2*segments,b+2*segments,c+2*segments,d+2*segments
        faces.extend([(a,c,b),(a,d,c),(A,B,C),(A,C,D),
                      (d,D,C),(d,C,c),(a,B,A),(a,b,B)])
    lines = ['solid rack_ring']
    for f in faces:
        lines += ['facet normal 0 0 0', 'outer loop']
        lines += ['vertex '+' '.join(f'{x:.10g}' for x in vertices[k]) for k in reversed(f)]
        lines += ['endloop', 'endfacet']
    lines += ['endsolid rack_ring']
    Path(path).write_text('\n'.join(lines)+'\n')


class Builder:
    def __init__(self):
        self.root=ET.Element('sdf',version='1.7');self.model=ET.SubElement(self.root,'model',name='lab_workbench')
        ET.SubElement(self.model,'static').text='true';self.link=ET.SubElement(self.model,'link',name='fixtures')
        self.items=[]

    def add(self,name,kind,dims,xyz,color,rpy=(0,0,0),uri=None):
        if name in {x['name'] for x in self.items}:raise ValueError('Duplicate fixture '+name)
        if any(float(x)<=0 for x in dims):raise ValueError('Nonpositive geometry '+name)
        self.items.append(dict(name=name,kind=kind,dimensions=list(dims),position=list(xyz),color=list(color)))
        for tag in ('visual','collision'):
            part=ET.SubElement(self.link,tag,name=name+'_'+tag)
            ET.SubElement(part,'pose').text=' '.join(str(float(x)) for x in (*xyz,*rpy))
            geom=ET.SubElement(part,'geometry');shape=ET.SubElement(geom,kind)
            if kind=='box':ET.SubElement(shape,'size').text=' '.join(map(str,dims))
            elif kind=='cylinder':
                ET.SubElement(shape,'radius').text=str(dims[0]);ET.SubElement(shape,'length').text=str(dims[1])
            elif kind=='sphere':ET.SubElement(shape,'radius').text=str(dims[0])
            elif kind=='mesh':ET.SubElement(shape,'uri').text=uri
            else:raise ValueError(kind)
            if tag=='visual':
                mat=ET.SubElement(part,'material')
                for field in ('ambient','diffuse'):ET.SubElement(mat,field).text=' '.join(map(str,color))
                ET.SubElement(mat,'specular').text='0.08 0.08 0.08 1'

    def box(self,name,dims,xyz,color):self.add(name,'box',dims,xyz,color)
    def cylinder(self,name,r,h,xyz,color):self.add(name,'cylinder',(r,h),xyz,color)
    def sphere(self,name,r,xyz,color):self.add(name,'sphere',(r,),xyz,color)


def build_lab_scene(cfg, mesh_dir):
    b=Builder();bench=cfg['bench'];cx,cy=bench['center_xy'];sx,sy=bench['size_xy'];height=bench['height'];thick=bench['thickness']
    b.box('bench_top',(sx,sy,thick),(cx,cy,-thick/2),(.68,.71,.73,1))
    for ix in (-1,1):
        for iy in (-1,1):
            b.box(f'bench_leg_{ix}_{iy}',(.055,.055,height-thick),(cx+ix*(sx/2-.075),cy+iy*(sy/2-.075),-(height+thick)/2),(.19,.23,.27,1))
    b.box('bench_front_brace',(sx-.1,.035,.06),(cx,cy-sy/2+.075,-height+.15),(.25,.29,.32,1))
    b.box('robot_mount_plate',(.20,.20,.008),(0,0,-.004),(.22,.25,.28,1))
    for rack in cfg['racks']:
        name=rack['name'];x,y=rack['center_xy'];rows=rack['rows'];cols=rack['columns'];pitch=rack['pitch'];rail=rack['rail_width'];top=rack['top_height'];color=rack['color'];width=cols*pitch+rail;depth=rows*pitch+rail
        b.box(name+'_base',(width,depth,.008),(x,y,.004),color)
        for dx in (-1,1):
            for dy in (-1,1):b.cylinder(name+f'_post_{dx}_{dy}',.006,top,(x+dx*(width/2-.005),y+dy*(depth/2-.005),top/2),color)
        for level,z in enumerate((top*.45,top)):
            for i in range(cols+1):b.box(name+f'_rail_x_{level}_{i}',(rail,depth,.006),(x+(i-cols/2)*pitch,y,z),color)
            for j in range(rows+1):b.box(name+f'_rail_y_{level}_{j}',(width,rail,.006),(x,y+(j-rows/2)*pitch,z),color)
        uri=(Path(mesh_dir)/(name+'_ring.stl')).resolve().as_uri()
        for i in range(cols):
            for j in range(rows):
                tx=x+(i-(cols-1)/2)*pitch;ty=y+(j-(rows-1)/2)*pitch;key=name+f'_{i}_{j}'
                b.add(key+'_socket','mesh',(rack['ring_radius']*2,rack['ring_radius']*2,.006),(tx,ty,top),color,uri=uri)
                # Leave two sockets empty to expose the actual aperture.
                if (i,j) in ((0,0),(cols-1,rows-1)):continue
                r=rack['tube_radius'];h=rack['tube_height']+.012*((i+j)%3)
                b.cylinder(key+'_tube',r,h-.02,(tx,ty,.02+(h-.02)/2),(.73,.84,.89,1))
                b.sphere(key+'_round_bottom',r,(tx,ty,.02),(.73,.84,.89,1))
                b.cylinder(key+'_cap',r*1.12,.014,(tx,ty,h+.007),CAPS[(i+j)%len(CAPS)])
    for bottle in cfg['bottles']:
        name=bottle['name'];x,y=bottle['xy'];r=bottle['radius'];h=bottle['height'];color=bottle['color']
        b.cylinder(name+'_body',r,h,(x,y,h/2),color)
        b.sphere(name+'_shoulder',r,(x,y,h),color)
        b.cylinder(name+'_neck',r*.43,.035,(x,y,h+r),color)
        b.cylinder(name+'_cap',r*.55,.018,(x,y,h+r+.026),(.12,.15,.18,1))
        b.box(name+'_label',(r*1.35,.002,h*.45),(x,y-r-.001,h*.52),(.93,.91,.82,1))
    tray=cfg['tray'];x,y=tray['center_xy'];sx,sy=tray['size_xy'];h=tray['rim_height'];tc=(.27,.43,.51,1)
    b.box('tray_bottom',(sx,sy,.006),(x,y,.003),tc)
    for sign in (-1,1):
        b.box('tray_wall_x_'+str(sign),(.006,sy,h),(x+sign*(sx/2-.003),y,h/2),tc)
        b.box('tray_wall_y_'+str(sign),(sx,.006,h),(x,y+sign*(sy/2-.003),h/2),tc)
    for i in range(3):
        for j in range(2):
            tx=x+(i-1)*.065;ty=y+(j-.5)*.085;h=.055+.015*j
            b.cylinder(f'vial_{i}_{j}',.018,h,(tx,ty,.006+h/2),(.84,.86,.81,1))
            b.cylinder(f'vial_cap_{i}_{j}',.020,.012,(tx,ty,.006+h+.006),CAPS[i+j])
    x,y=cfg['flask_xy']
    b.cylinder('flask_base',.045,.025,(x,y,.0125),(.88,.68,.25,1))
    # Stacked cylinders approximate an opaque flask with genuinely varying width.
    for i in range(6):b.cylinder('flask_taper_'+str(i),.045-i*.0047,.018,(x,y,.025+(i+.5)*.018),(.88,.68,.25,1))
    b.cylinder('flask_neck',.018,.045,(x,y,.1555),(.88,.68,.25,1))
    for obj in cfg['blocks']:b.box(obj['name'],obj['size'],obj['xyz'],obj['color'])
    return ET.tostring(b.root,encoding='unicode'),dict(bench_top_z=0.,floor_z=-height,fixtures=b.items,rack_count=len(cfg['racks']),tube_count=sum(1 for x in b.items if x['name'].endswith('_tube')),opaque_materials=True)
