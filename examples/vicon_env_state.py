import numpy as np
import time
import pinocchio as pin
import motioncapture



class ViconEnvState:

    def __init__(
        self, system_type="vicon", hostname="192.168.2.70", T_vicon_to_robot=pin.SE3.Identity() , D_ref_frame_wrt_vicon_frame = {}):
        self.mc = motioncapture.connect(system_type, {"hostname": hostname})
        self.state = None
        self.T_vicon_to_robot = T_vicon_to_robot
        self.D_ref_frame_wrt_vicon_frame = D_ref_frame_wrt_vicon_frame
        for i in range(100):
            self.mc.waitForNextFrame()
            time.sleep(.001)
       

    def get_state(self):
        self.mc.waitForNextFrame()        
        D = {}
        for name, obj in self.mc.rigidBodies.items():
            # print("name", name)
            quat = pin.Quaternion(w=obj.rotation.w, x=obj.rotation.x, y=obj.rotation.y, z=obj.rotation.z)
            pos = obj.position
            T = pin.SE3(quat, pos)
            # print("original T", T)

            if name in self.D_ref_frame_wrt_vicon_frame:
                #print(" self.D_ref_frame_wrt_vicon_frame[name]", self.D_ref_frame_wrt_vicon_frame[name]) 
                T =  T  * self.D_ref_frame_wrt_vicon_frame[name] 
                # print("T after")
                # print(T)

                # TODO: check the math here!
            T = self.T_vicon_to_robot * T # transform to the robot frame
            # print("T final")
            # print(T)
            D[name] = T
           
            
        return D
