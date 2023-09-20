#!/usr/bin/env python3
#Rahman ve Rahim Olan Allah'ın adıyla
import numpy as np
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.envs.VelocityAviary import VelocityAviary
import pybullet as p
import rospy
from SimulationParameter import *


from GammaSwarm.srv import ServiceMessage


class Environment:
    def __init__(self):
        self.params = SimulationParameter()
        self.AGGR_PHY_STEPS = int(
            self.params.simulation_freq_hz / self.params.control_freq_hz) if self.params.aggregate else 1
        #BU BÖLÜMDE Initial/ nodeuna basılan listeye subscribe olunacak
        #buraya konum gelecek
        
        #rospy.init_node("environment", anonymous= True)
        #data = rospy.wait_for_message('Initial', FullState, timeout= 10)
        
        #INITIALIZING NUM_DRONES, XYZS AND RPYS
        self.service = None
        self.server_manual_shutdown = False
        
        self.params.num_drones = None
        self.params.INIT_XYZS = None #np.zeros((self.num_drones, 3))
        self.params.INIT_RPYS = None #np.zeros((self.num_drones, 3))

        self.server()


    def server(self):
        rospy.init_node("SIMULATION")
        self.service = rospy.Service("Simulation-Initializer-Server", ServiceMessage, self.initializeSim)
        r = rospy.Rate(10)
        while not self.server_manual_shutdown:
            r.sleep()
        
        self.service.shutdown()
        del self.service


    def initializeSim(self, request):
        print("Initializing Simulation")
        ##################################
        data = request.FullStateList
        self.params.num_drones = len(data)
        self.params.INIT_RPYS = np.zeros((self.params.num_drones,3))
        self.params.INIT_XYZS = np.zeros((self.params.num_drones,3))
        #print(self.params.INIT_XYZS)

        for i in range(self.params.num_drones):
            self.params.INIT_XYZS[i,0] = data[i].pose.position.x
            self.params.INIT_XYZS[i,1] = data[i].pose.position.y
            self.params.INIT_XYZS[i,2] = data[i].pose.position.z -0.08

            euler = p.getEulerFromQuaternion([data[i].pose.orientation.x, data[i].pose.orientation.y, data[i].pose.orientation.z, data[i].pose.orientation.w])

            self.params.INIT_RPYS[i,0] = euler[0]
            self.params.INIT_RPYS[i,1] = euler[1]
            self.params.INIT_RPYS[i,2] = euler[2]
        ##################################
        self.server_manual_shutdown = True
        return True
        
    

    # enter the type as starting formation type
    def properEnv(self):
        if self.params.vision:
            env = VisionAviary(drone_model=self.params.drone,
                               num_drones=self.params.num_drones,
                               initial_xyzs=self.params.INIT_XYZS,
                               initial_rpys=self.params.INIT_RPYS,
                               physics=self.params.physics,
                               neighbourhood_radius=self.params.neighbourhood_radius,
                               freq=self.params.simulation_freq_hz,
                               aggregate_phy_steps=self.AGGR_PHY_STEPS,
                               gui=self.params.gui,
                               record=self.params.record_video,
                               obstacles=self.params.obstacles
                               )
        else:
            env = VelocityAviary(drone_model=self.params.drone,
                             num_drones=self.params.num_drones,
                             initial_xyzs=self.params.INIT_XYZS,
                             initial_rpys=self.params.INIT_RPYS,
                             physics=self.params.physics,
                             neighbourhood_radius=self.params.neighbourhood_radius,
                             freq=self.params.simulation_freq_hz,
                             aggregate_phy_steps=self.AGGR_PHY_STEPS,
                             gui=self.params.gui,
                             record=self.params.record_video,
                             obstacles=self.params.obstacles,
                             user_debug_gui=self.params.user_debug_gui
                             )
        return env
