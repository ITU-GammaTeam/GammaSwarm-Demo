#RAHMAN VE RAHIM OLAN ALLAH'IN ADIYLA

"""
NOT USED BUT MAINTAIN FOR DEVELOPING, MAY BE USE!
For Questions:
@Author: Muhammed Emin Hamamcı: hamamci19@itu.edu.tr
"""


#Rahman ve Rahim Olan Allah'ın Adıyla
import numpy as np

class Position:
    def  __init__(self, x, y, z, roll = 0, pitch = 0 , yaw = 0):
        self.x = x
        self.y = y
        self.z = z
        self.roll =roll
        self.pitch = pitch
        self.yaw = yaw
    
    def toString(self):
        return "Position : " + "x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z)


class Orientation:
    def  __init__(self, x, y, z,w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def toString(self):
        return "Position : " + "x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z)



class Velocity:
    def __init__(self, x, y, z,wx,wy,wz):
        self.x = x
        self.y = y
        self.z = z
        self.wx = wx
        self.wy = wy
        self.wz = wz


    def toString(self):
        return "Velocity : " + "x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z)



class MellingerController:

    def __init__(self):
        self.quaternion_of_drones = Orientation(0,0,0,0)
        self.position_of_drones = Position(0,0,0,0,0,0)
        self.velocity_of_drones = Velocity(0,0,0,0,0,0)
        

    def compute_zb(self):
        quaternion_rotation_array = np.zeros((3,3))
        x = self.quaternion_of_drones.x
        y = self.quaternion_of_drones.y
        z = self.quaternion_of_drones.z
        w = self.quaternion_of_drones.w
        quaternion_rotation_array[0][0] = 1 - 2*y*y - 2*z*z
        quaternion_rotation_array[0][1] = 2*x*y - 2*z*w
        quaternion_rotation_array[0][2] = 2*x*z + 2*y*w
        quaternion_rotation_array[1][0] = 2*x*y + 2*z*w
        quaternion_rotation_array[1][1] = 1 - 2*x*x - 2*z*z
        quaternion_rotation_array[1][2] = 2*y*z - 2*x*w
        quaternion_rotation_array[2][0] = 2*x*z - 2*y*w
        quaternion_rotation_array[2][1] = 2*y*z + 2*x*w
        quaternion_rotation_array[2][2] = 1 - 2*x*x - 2*y*y
        zb = np.array([quaternion_rotation_array[0][2],quaternion_rotation_array[1][2],quaternion_rotation_array[2][2]]).T
        return zb

    def compute_yb(self,zb):
        xc = np.array([(np.cos(self.position_of_drones.yaw)),np.sin(self.position_of_drones.yaw),0]).T
        cross = np.cross(zb,xc).T
        norm = np.linalg.norm(cross)
        yb = cross/norm
        return yb
    def compute_xb(self,yb,zb):
        xb = np.cross(yb,zb)
        return xb
    def compute_control(self,target_position,target_velocity):
        epx = self.position_of_drones.x - target_position.x
        epy = self.position_of_drones.y - target_position.y
        epz = self.position_of_drones.z - target_position.z

        ep = np.array([epx,epy,epz]).T


        z_body = self.compute_zb()
        y_body = self.compute_yb(z_body)
        x_body = self.compute_xb(y_body,z_body)
        #Once direk takeoff height verecegim
        

a = MellingerController()
ay = Position(0,0,0,0,0,0)
#a.compute_control(ay)