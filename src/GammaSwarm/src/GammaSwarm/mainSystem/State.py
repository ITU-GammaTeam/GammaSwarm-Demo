#Rahman ve Rahim Olan Allah'ın Adıyla

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

class DesiredTrajectoryState:
    def __init__(self,desired_position, desired_velocity, desired_acceleration, desired_yaw, desired_omega):
        #Check it out, print on MerkezcilClass.py
        self.desired_position = desired_position #np.array 3-D
        self.desired_velocity = desired_velocity #np.array 3-D
        self.desired_acceleration = desired_acceleration #np.array 3-D
        self.desired_yaw = desired_yaw #Float
        self.desired_omega = desired_omega #np.array 3-D 


    def toString(self):
        return "Velocity : " + "x : " + str(self.x) + " y : " + str(self.y) + " z : " + str(self.z)