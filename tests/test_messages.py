"""Wire equivalence, including the local Humble Marker schema with texture fields."""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/ur7e_nvblox_in_hand'))
from ur7e_nvblox_in_hand import messages
from rclpy.serialization import serialize_message,deserialize_message
from visualization_msgs.msg import MarkerArray

class WireTests(unittest.TestCase):
    def test_mesh_wire_equivalence(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],np.float32)
        c=np.array([[255,0,0],[0,255,0],[0,0,255],[127,128,129]],np.uint8)
        for frame in ['base_link','map','长名称坐标系']:
            for triangles in [np.empty((0,3),np.int32),np.array([[0,1,2]],np.int32),np.array([[0,1,2],[1,2,3]],np.int32)]:
                for stamp in [0,123456789123456789]:
                    data=(v,triangles,c)
                    raw=messages.mesh_serialized(data,frame,stamp)
                    expected=serialize_message(messages.mesh_markers(data,frame,stamp))
                    self.assertEqual(raw,expected)
                    parsed=deserialize_message(raw,MarkerArray)
                    self.assertEqual(len(parsed.markers[0].points),3*len(triangles))
                    self.assertEqual(parsed.markers[0].action,0 if len(triangles) else 2)
    def test_esdf_contract(self):
        a=np.array([[.015,.015,.015,-.03,1,1,1],[.045,.015,.015,.06,1,0,0]],np.float32)
        m=messages.esdf_cloud(a,'base_link',123)
        self.assertEqual(m.point_step,24);self.assertEqual(len(m.data),48)
        b=np.frombuffer(m.data,np.uint8).reshape(-1,24)
        self.assertEqual(b[:,20].tolist(),[1,1]);self.assertEqual(b[:,21].tolist(),[1,0])
        self.assertEqual(m.header.stamp.nanosec,123)

if __name__=='__main__':unittest.main()
