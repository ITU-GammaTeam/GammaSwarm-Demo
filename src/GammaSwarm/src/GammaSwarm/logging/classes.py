class Point:
    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    def toStr(self):
        return "x: " + str(round(self.x,3)) + " y: " + str(round(self.y,3)) + " z: " + str(round(self.z,3))



class Quaternion:
    def __init__(self, x=None, y=None, z=None, w=None):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def toStr(self):
        return "x: " + str(round(self.x,3)) + " y: " + str(round(self.y,3)) + " z: " + str(round(self.z,3)) + " w: " + str(round(self.w,3))


class Vector3:
    def __init__(self, x=None, y=None, z=None):
        self.x = x
        self.y = y
        self.z = z

    def toStr(self):
        return "x: " + str(round(self.x,3)) + " y: " + str(round(self.y,3)) + " z: " + str(round(self.z,3))

class Pose:
    def __init__(self, position=None, orientation=None):
        self.position = position
        self.orientation = orientation

class Twist:
    def __init__(self, linear=None, angular=None):
        self.linear = linear
        self.angular = angular


class State:
    def __init__(self, id=None, active=False, pose=None, twist=None, accelaeration=None):
        self.id = id
        self.active = active
        self.pose = pose
        self.twist = twist
        self.accelaeration = accelaeration

class Command:
    def __init__(self, id=None, active=False, twist=None):
        self.id = id
        self.active = active
        self.twist = twist

class TrajectoryCommand:
    def __init__(self, id=None, active=False, pose=None, twist=None, accelaeration=None, yaw=None):
        self.id = id
        self.active = active
        self.pose = pose
        self.twist = twist
        self.accelaeration = accelaeration
        self.yaw = yaw


class Sequance():
    def __init__(self, no, logs,uav_names):
        self.no = no
        self.seq_time = None
        self.states = {}
        self.commands = {}
        self.t_commands = {}
        self.all_logs = logs
        self.uav_names = uav_names
        self.parse_logs(logs)

    def parse_logs(self, logs):
        for log in logs:
            self.seq_time = log["time"]
            if log["type"] == "state":
                self.states[log["data"].id] = log["data"]
            elif log["type"] == "command":
                self.commands[log["data"].id] = log["data"]
            elif log["type"] == "t_command":
                self.t_commands[log["data"].id] = log["data"]

