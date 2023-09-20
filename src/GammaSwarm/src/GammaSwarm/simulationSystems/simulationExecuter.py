#!/usr/bin/env python3

#Rahman ve Rahim olan Allah'in Adıyla - 2022
#Rahman ve Rahim Olan Allah'in Adiyla - 2023 Bismillah
import pybullet as p
import time

import rospy
from GammaSwarm.msg import FullCommand
from GammaSwarm.msg import FullState
from GammaSwarm.msg import UavState
from GammaSwarm.msg import FullTrajectoryCommand

from gym_pybullet_drones.utils.utils import sync
from environment import *
import numpy as np
from ControllerUtils import *



"""

In this code simulation execution done. Simulation execution have:

    Manage all in coming data and out data. This means simulation manager publish current telemetry data of drones
    and subscribe command coming from mainSystem, pls look at mainSystem code for more information.

    And using command data gives commands to drone using gym_pybullet env.step() function with action List matrix.
    For more information pls look at gym_pybullet repository : https://github.com/utiasDSL/gym-pybullet-drones

    @Author: Muhammed Emin Hamamcı - hamamci19@itu.edu.tr
    @Contributor: Muhammed Emin Hamamcı, Yunus Emre Yiğit and others members of ITU-GAMMA Team 2022

"""


class SimulationManager:
    
    def __init__(self): 
        self.command = None
        self.pub = rospy.Publisher("simulation_uav_state", FullState, queue_size = 10)
        
        
    def subscriber(self):
        rospy.Subscriber("uav_commands", FullCommand, self.call_back)
        rospy.Subscriber("uav_trajectory_commands", FullTrajectoryCommand, self.trajectory_call_back)
    

    def call_back(self, data): #Callback for FullCommand message.
        self.command = data


    def trajectory_call_back(self, data): #Callback for FullTrajectoryCommand message.
        self.command = data


    def publisher(self, uav_states): #Publish to FullStateMessage. Generated in Loop.
        self.pub.publish(uav_states)





if __name__ == "__main__":
    env_class = Environment()
    env = env_class.properEnv()
    manager = SimulationManager()
    mellinger = MellingerController()
    

    PYB_CLIENT = env.getPyBulletClient()
    DRONE_IDS = env.getDroneIds()
    actionL = {str(i): np.array([0, 0, 0, 0]) for i in range(env_class.params.num_drones)}
    print(actionL)
    target_angular_rate = {str(i): np.array([0, 0, 0]) for i in range(env_class.params.num_drones)}
    initial_obs = env._computeObs()
    #####################################Run the simulation ####################################

    START = time.time()


    manager.subscriber() #After initialization Subscribe to commands.
    

    freq = rospy.Rate(60)
    #TODO Emir verme 15 Hz için yapılabilir. Denenebilir.
    while not rospy.is_shutdown():
        
        command = manager.command
        
        #Manager function may be return non-value in first(0-10) loops.
        if command == None:
            #print("Command is none")
            continue
        
        _, _, _, _ = env.step(actionL,target_angular_rate)
        obs = env._computeObs()

        velocity_values = []
        angular_values = []
        for i in range(env_class.params.num_drones):
            #print("DRONE IDS ====="+str(DRONE_IDS))
            cmd_msg = manager.command.allcommands[i]
            virtual_drone = obs[str(i)]
            state=virtual_drone["state"]
            
            act_roll = state[0]
            act_pitch = state[1]
            act_yaw = state[2]
            
            #Defining Rotation Matrix
            row1 = [(np.cos(act_pitch)*np.cos(act_yaw)),(np.cos(act_pitch)*np.sin(act_yaw)),(-1*np.sin(act_pitch))]
            row2 = [((-1*np.cos(act_roll)*np.sin(act_yaw))+(np.sin(act_roll)*np.sin(act_pitch)*np.cos(act_yaw))),     ((np.cos(act_roll)*np.cos(act_yaw))+(np.sin(act_roll)*np.sin(act_pitch)*np.sin(act_yaw))),      (np.sin(act_roll)*np.cos(act_pitch))]
            row3 = [((np.sin(act_roll)*np.sin(act_yaw))+(np.cos(act_roll)*np.sin(act_pitch)*np.cos(act_yaw))),      ((-1*np.sin(act_roll)*np.cos(act_yaw))+(np.cos(act_roll)*np.sin(act_pitch)*np.sin(act_yaw))),     (np.cos(act_roll)*np.cos(act_pitch))]
            R_body_to_world = np.array([row1,row2,row3]).T
            
            msg_array = np.array([cmd_msg.twist.angular.x,cmd_msg.twist.angular.y,cmd_msg.twist.angular.z])
            angular_vel = R_body_to_world@msg_array
            angular_values.append(np.array(angular_vel))

            target_vel = np.array([cmd_msg.twist.linear.x*((60*60)/1000),cmd_msg.twist.linear.y*((60*60)/1000),(cmd_msg.twist.linear.z)*((60*60)/1000)])
            target_vel_kmh_fraction = (np.linalg.norm(target_vel)*((60*60)/1000))/30.0
            velocity_values.append(np.append(target_vel,target_vel_kmh_fraction))

        actionL = dict(list(enumerate(velocity_values)))
        target_angular_rate = dict(list(enumerate(angular_values)))


        if i % env.SIM_FREQ == 0:
            env.render()
            #### Print matrices with the images captured by each drone #
            if env_class.params.vision:
                for j in range(env_class.params.num_drones):
                    print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),
                          obs[str(j)]["dep"].shape, np.average(obs[str(j)]["dep"]),
                          obs[str(j)]["seg"].shape, np.average(obs[str(j)]["seg"])
                          )
        
        state_msg = FullState()
        for i in range(env_class.params.num_drones):
            msg = UavState()
            virtual_drone = obs[str(i)]
            state=virtual_drone["state"]
            
            #In simulation drones always active! In real this is not! Look at realSystems.
            msg.active = True
            msg.id = "uav/"+str(i+1)
            msg.pose.position.x = state[0]
            msg.pose.position.y = state[1]
            msg.pose.position.z = state[2]

            msg.pose.orientation.x = state[3]
            msg.pose.orientation.y = state[4]
            msg.pose.orientation.z = state[5]
            msg.pose.orientation.w = state[6]

            msg.twist.linear.x = state[10]
            msg.twist.linear.y = state[11]
            msg.twist.linear.z = state[12]

            msg.twist.angular.x = state[13]
            msg.twist.angular.y = state[14]
            msg.twist.angular.z = state[15]
            state_msg.fullState.append(msg)

        
        manager.publisher(state_msg)
        freq.sleep()
        
        if env_class.params.gui:
            sync(i, START, env.TIMESTEP)
            
    env.close()








#########################################################  DEVELOPING  ###############################################################################


"In this loop try to achieve Mellinger Controller for simulation. But it took a long effort to do this. For this reason, this work has been suspended."

# for i in range(env_class.params.num_drones):
        #     #LOCK ACQUIRE YAZILABİLİR!!!
        #     #DRONE_IDS = manager.allcommands[i].id[4:]
        #     #print("DRONE IDS ====="+str(DRONE_IDS))
        #     cmd_msg = manager.command.allcommands[i]
        #     #print(cmd_msg)
        #     virtual_drone = obs[str(i)]
        #     state=virtual_drone["state"]
            
        #     act_roll = state[0]
        #     act_pitch = state[1]
        #     act_yaw = state[2]
        #     #Defining Rotation Matrix
        #     row1 = [(np.cos(act_pitch)*np.cos(act_yaw)),(np.cos(act_pitch)*np.sin(act_yaw)),(-1*np.sin(act_pitch))]
        #     row2 = [((-1*np.cos(act_roll)*np.sin(act_yaw))+(np.sin(act_roll)*np.sin(act_pitch)*np.cos(act_yaw))),     ((np.cos(act_roll)*np.cos(act_yaw))+(np.sin(act_roll)*np.sin(act_pitch)*np.sin(act_yaw))),      (np.sin(act_roll)*np.cos(act_pitch))]
        #     row3 = [((np.sin(act_roll)*np.sin(act_yaw))+(np.cos(act_roll)*np.sin(act_pitch)*np.cos(act_yaw))),      ((-1*np.sin(act_roll)*np.cos(act_yaw))+(np.cos(act_roll)*np.sin(act_pitch)*np.sin(act_yaw))),     (np.cos(act_roll)*np.cos(act_pitch))]
        #     R_body_to_world = np.array([row1,row2,row3]).T
            
        #     msg_array = np.array([cmd_msg.twist.angular.x,cmd_msg.twist.angular.y,cmd_msg.twist.angular.z])
        #     angular_vel = R_body_to_world@msg_array
        #     #print(angular_vel[2],cmd_msg.twist.angular.z)
        #     p.resetBaseVelocity(DRONE_IDS[i],
        #                         [cmd_msg.twist.linear.x,cmd_msg.twist.linear.y,cmd_msg.twist.linear.z+0.125],
        #                         [0,0,angular_vel[2]],
        #                         physicsClientId=PYB_CLIENT
        #                         )
            
            #mellinger.quaternion_of_drones.x = state[3]
            #mellinger.quaternion_of_drones.y = state[4]
            #mellinger.quaternion_of_drones.z = state[5]
            #mellinger.quaternion_of_drones.w = state[6]
#
            #mellinger.position_of_drones.x = state[0]
            #mellinger.position_of_drones.y = state[1]
            #mellinger.position_of_drones.z = state[2]
#
            #mellinger.position_of_drones.roll = state[0]
            #mellinger.position_of_drones.pitch = state[1]
            #mellinger.position_of_drones.yaw = state[2]
#
            #mellinger.velocity_of_drones.x = state[10]
            #mellinger.velocity_of_drones.y = state[11]
            #mellinger.velocity_of_drones.z = state[12]
            #mellinger.velocity_of_drones.wx = state[13]
            #mellinger.velocity_of_drones.wy = state[14]
            #mellinger.velocity_of_drones.wz = state[15]
#
            #target_takeoff_position = Position(initial_obs[str(i)][0],initial_obs[str(i)][1],takeoff_params.takeoff_height)
            #target_velocity = Velocity(cmd_msg.twist.linear.x,cmd_msg.twist.linear.y,cmd_msg.twist.linear.z,cmd_msg.twist.angular.x,cmd_msg.twist.angular.y,cmd_msg.twist.angular.z)
            #mellinger.compute_control(target_takeoff_position,target_velocity)




