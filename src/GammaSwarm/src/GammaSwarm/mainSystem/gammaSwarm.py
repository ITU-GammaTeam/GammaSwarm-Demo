#RAHMAN VE RAHIM OLAN ALLAH'IN ADIYLA, MUVAFFAKIYETMIZ YALNIZCA O'NA AITTIR!
import rospy
import numpy as np
from State import Position,Velocity,DesiredTrajectoryState
from UavClass import UavClass
import pybullet as p
from threading import Lock
from GammaSwarm.msg import FullState, FullCommand, FullTrajectoryCommand
from geometry_msgs.msg import Pose


class GammaSwarm:
    def __init__(self,init_params,initial_position):
        self.init_params = init_params
        self.uav_count = self.init_params.number_of_agent
        self.uav_list = []
        self.initial_position_list = initial_position["PoseArray"]
        self.initial_orientation_list = initial_position["OrientationArray"]
        self.mutex = Lock()
        points = np.array(self.initial_position_list)
        center = np.average(points,axis=0)
        self.swarm_center = Position(center[0],center[1],center[2])
        
        self.initialize()

        
    def initialize(self):
        rospy.init_node('GammaSwarm')
        self.pub = rospy.Publisher("uav_commands",FullCommand,queue_size=10)
        self.trajectory_pub = rospy.Publisher("uav_trajectory_commands", FullTrajectoryCommand, queue_size=10)
        for i in range(self.uav_count):
            uav_class = UavClass("uav/"+str(i+1),self.initial_position_list[i],self.initial_orientation_list[i])
            self.uav_list.append(uav_class)


        #If and only simulation enable, subscribe simulation data.
        if self.init_params.simulation_enabled and not self.init_params.real_enabled:
            rospy.Subscriber("simulation_uav_state",FullState,self.simStateCallback)

        #If and only real enabe, subscribe real data.
        elif not self.init_params.simulation_enabled and self.init_params.real_enabled:
            rospy.Subscriber("real_uav_state", FullState, self.realStateCallback)



    def simStateCallback(self,data):
        self.mutex.acquire()
        for msg in data.fullState:
            index = int(msg.id[4:])-1
            euler = p.getEulerFromQuaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            position = Position(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,euler[0],euler[1],euler[2])
            velocity = Velocity(msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z)
            self.uav_list[index].activation_flag = msg.active
            self.uav_list[index].current_velocity = velocity
            self.uav_list[index].current_position = position
        self.mutex.release()
        


    def realStateCallback(self,data):
        self.mutex.acquire()
        for msg in data.fullState:
            index = int(msg.id[4:])-1
            euler = p.getEulerFromQuaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            position = Position(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,euler[0],euler[1],euler[2])
            velocity = Velocity(msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z)
            self.uav_list[index].activation_flag = msg.active
            self.uav_list[index].current_velocity = velocity
            self.uav_list[index].current_position = position
        self.mutex.release()



    def publishCommand(self):
        msg = FullCommand()
        for uav in self.uav_list:
            msg.allcommands.append(uav.command_message)
        self.pub.publish(msg)

        
    def publishTrajectoryCommand(self):
        msg = FullTrajectoryCommand()
        for uav in self.uav_list:
            msg.allcommands.append(uav.trajectory_command_message)
        self.trajectory_pub.publish(msg)


    def update_SwarmCenter(self):
        #TODO must be update if drones active!!!!!
        active_pose_list = []
        for uav in self.uav_list:
            #print("ACTIVE",uav.id,uav.activation_flag)
            if uav.activation_flag == True:
                active_pose_list.append([uav.current_position.x,uav.current_position.y,uav.current_position.z]) 

        points = np.array(active_pose_list)
        center = np.average(points,axis=0)
        self.swarm_center = Position(center[0],center[1],center[2])
        del active_pose_list
    
