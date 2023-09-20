#Rahman ve Rahim Olan Allah'ın Adıyla
"""
@Main Author: Yunus Emre Yiğit -yigity19
@Contributor: Muhammed Emin Hamamcı, Muhammed Veli Ünlü, Umut Gökdağ ?
"""
import rospy
import numpy as np
from State import *
from initializer_util import *

import time
import rospy
from GammaSwarm.msg import UavState
from GammaSwarm.srv import ServiceMessage
from GammaSwarm.srv import ServiceMessageRequest
from GammaSwarm.srv import RealServiceMessage, RealServiceMessageResponse

import pybullet as p



"""
#TODO: It should be made more organized
The purpose of the below functions is to generate starting positions for the simulation.

"""

def convertINITToArray(init):
        INIT_XYZS = []
        for ele in init:
            INIT_XYZS.append([ele.x, ele.y, ele.z])
        INIT_XYZS = np.array(INIT_XYZS)
        return INIT_XYZS
    

def calculateInitialPose(init_params):
    
    formation = Formation()

    INIT_RPYS = np.array([[0, 0, i * (np.pi / 2) / init_params.number_of_agent] for i in range(init_params.number_of_agent)])
    
    
    H = .1
    R = init_params.number_of_agent / 5.

    radIntPoints = np.array(
            [[R * np.cos((i / init_params.number_of_agent) * 2 * np.pi + np.pi / 2),
              R * np.sin((i / init_params.number_of_agent) * 2 * np.pi + np.pi / 2) - R,
              H] for i in range(init_params.number_of_agent)])
    
    poses = []
    for ele in radIntPoints:
        poses.append(Position(ele[0], ele[1], ele[2]))

    if init_params.starting_formation == "common":
        formation.commonFormationPoints(poses, Position(0, 0, H))
        INIT_XYZS = formation.formationPoints
        INIT_XYZS = convertINITToArray(INIT_XYZS)
    init_state_dict = {"PoseArray": INIT_XYZS,"OrientationArray":INIT_RPYS}

    return init_state_dict



#Rahman ve Rahim Olan Allah'ın Adıyla
#FORM A CLIENT IN ORDER TO REQUEST TO FORM SIMULATION SENDING THE  INITIAL POSITIONS OF DRONES TO THE SIMULATION FROM "SIMULATION_FORM" SERVER
def systemInitializer(init_params):
    if init_params.simulation_enabled and not init_params.real_enabled:
        
        """
        @In this block, if we only want to run a simulation, the init pose and init orientations 
        required to set up the simulation are calculated and published. 
        Then, when the initialization mode changes, these nodes should be closed.
        """

        initial_state_dict = calculateInitialPose(init_params)
        
        rospy.wait_for_service("Simulation-Initializer-Server",timeout=10)
        
        initial_position_list = initial_state_dict["PoseArray"]
        initial_orientation_list = initial_state_dict["OrientationArray"]


        #FullState service message object formed  
        initial_pose_msg_list = ServiceMessageRequest()

        for i in range(0,init_params.number_of_agent):
            msg = UavState()
            uav_pose = initial_position_list[i]
            uav_orientation = initial_orientation_list[i]
            quaternion = p.getQuaternionFromEuler([uav_orientation[0], uav_orientation[1], uav_orientation[2]])

            msg.id = "uav/"+str(i+1)
            msg.pose.position.x = uav_pose[0]
            msg.pose.position.y = uav_pose[1]
            msg.pose.position.z = uav_pose[2]
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]

            initial_pose_msg_list.FullStateList.append(msg)
            
        try:
            initializer = rospy.ServiceProxy("Simulation-Initializer-Server", ServiceMessage)
            response_from_server = initializer(initial_pose_msg_list)
            print("SIMULASYON DURUMU : ", response_from_server)
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed. Server sent the error: ", e)
            raise e

        return initial_state_dict
    

    elif not init_params.simulation_enabled and init_params.real_enabled:

        """
        @In this block, if we just want to run it for real, the necessary initial position data is taken and given to the system. 
        Then, when the initialization mode changes, these nodes must be closed.        
        """

        rospy.wait_for_service("Real-Initializer-Server",timeout=10)
        
        try:
            initializer = rospy.ServiceProxy("Real-Initializer-Server", RealServiceMessage)
            response_from_server = initializer(True)
            response_from_server = response_from_server.FullStateList
            print(response_from_server)
            pose = []
            orient = []
            for message in response_from_server:
                pose.append([message.pose.position.x,message.pose.position.y,message.pose.position.z])
                orient.append([message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w])

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed. Server sent the error: ", e)
            raise e
        initial_state_dict = {"PoseArray" : pose, "OrientationArray": orient}
        return initial_state_dict
        
            
    
