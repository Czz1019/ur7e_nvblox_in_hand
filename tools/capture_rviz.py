"""Capture only the RViz backing pixmap; never fall back to the foreground desktop."""
import ctypes as C
import re
import subprocess
import sys
from PIL import Image

class XImage(C.Structure):
    _fields_=[('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),('format',C.c_int),
      ('data',C.c_void_p),('byte_order',C.c_int),('bitmap_unit',C.c_int),
      ('bitmap_bit_order',C.c_int),('bitmap_pad',C.c_int),('depth',C.c_int),
      ('bytes_per_line',C.c_int),('bits_per_pixel',C.c_int),
      ('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong),('obdata',C.c_void_p)]
class Visual(C.Structure):
    _fields_=[('ext_data',C.c_void_p),('visualid',C.c_ulong),('visual_class',C.c_int),
      ('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong),
      ('bits_per_rgb',C.c_int),('map_entries',C.c_int)]

tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
matches=[(wid,title) for wid,title in re.findall(r'(0x[0-9a-f]+) "([^"\n]*RViz[^"\n]*)"',tree)
         if 'ur7e_nvblox_in_hand' in title]
if len(matches)!=1:raise RuntimeError('Expected exactly one owned test RViz window: '+str(matches))
x=C.cdll.LoadLibrary('libX11.so.6');composite=C.cdll.LoadLibrary('libXcomposite.so.1')
x.XOpenDisplay.restype=C.c_void_p
x.XCloseDisplay.argtypes=[C.c_void_p]
composite.XCompositeNameWindowPixmap.argtypes=[C.c_void_p,C.c_ulong]
composite.XCompositeNameWindowPixmap.restype=C.c_ulong
composite.XCompositeRedirectWindow.argtypes=[C.c_void_p,C.c_ulong,C.c_int]
x.XSync.argtypes=[C.c_void_p,C.c_int]
x.XGetGeometry.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_int),C.POINTER(C.c_int),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint)]
x.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int]
x.XGetImage.restype=C.POINTER(XImage)
x.XDestroyImage.argtypes=[C.POINTER(XImage)]
x.XFreePixmap.argtypes=[C.c_void_p,C.c_ulong]
d=x.XOpenDisplay(None)
if not d:raise RuntimeError('No X display')
wid=int(matches[0][0],16)
# Some desktops do not run a compositor. Automatic redirection keeps the
# window displayed normally and is removed when this short-lived client exits.
composite.XCompositeRedirectWindow(d,wid,0)
x.XSync(d,0)
pix=composite.XCompositeNameWindowPixmap(d,wid)
root=C.c_ulong();a=C.c_int();b=C.c_int();w=C.c_uint();h=C.c_uint();border=C.c_uint();depth=C.c_uint()
x.XGetGeometry(d,pix,C.byref(root),C.byref(a),C.byref(b),C.byref(w),C.byref(h),C.byref(border),C.byref(depth))
im=x.XGetImage(d,pix,0,0,w.value,h.value,C.c_ulong(-1).value,2)
if not im:raise RuntimeError('Cannot read RViz backing pixmap')
v=im.contents
print('XImage layout:',v.width,v.height,v.bits_per_pixel,v.byte_order,hex(v.red_mask),hex(v.green_mask),hex(v.blue_mask),flush=True)
if v.red_mask==0 and v.green_mask==0 and v.blue_mask==0:
    x.XDefaultVisual.argtypes=[C.c_void_p,C.c_int];x.XDefaultVisual.restype=C.POINTER(Visual)
    visual=x.XDefaultVisual(d,0).contents
    red,blue=visual.red_mask,visual.blue_mask
else:red,blue=v.red_mask,v.blue_mask
if v.bits_per_pixel!=32 or v.byte_order!=0 or red!=0xff0000 or blue!=0xff:
    raise RuntimeError('Unsupported X image layout')
raw=C.string_at(v.data,v.bytes_per_line*v.height)
Image.frombytes('RGB',(v.width,v.height),raw,'raw','BGRX',v.bytes_per_line,1).save(sys.argv[1])
x.XDestroyImage(im);x.XFreePixmap(d,pix);x.XCloseDisplay(d)
print('RViz backing pixmap captured:',sys.argv[1],w.value,h.value)
