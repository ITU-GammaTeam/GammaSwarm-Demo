#!/usr/bin/env python3
import rospy , rostopic
import logging 
from datetime import datetime
from multiprocessing import Process
import os
from GammaSwarm.msg import FullCommand , FullState , FullTrajectoryCommand
import time

rospy.init_node("GammaLogger")

class GammaLogger:
    def __init__(self):
        self.flight_starting_time = time.time()
        self.logger_id = str(datetime.now()).replace(" ","-").replace(":","-")
        self.parent_folder = "gamma_logs"
        self.vehicle_numbers = (5,5)
        self.rate = rostopic.ROSTopicHz(-1)  
        self.simulation_enebled = False
        self.real_enebled = True
        self.formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        self.uav_loggers = {}
        self.ugv_loggers = {}
        self.uav_real_state_data = {}
        self.uav_sim_state_data = {}
        self.uav_command_data = {}
        self.uav_traj_command_data = {}
        self.ugv_data = {}
        self.createLoggers(self.parent_folder,self.logger_id,self.vehicle_numbers)

        #list of names of topics/services to be waited 
        self.subscribeRosTopics()
        self.process1 = Process(target=self.log_commands)
        self.process2 = Process(target=self.log_rates)
        self.process3 = Process(target=self.log_states)




    def subscribeRosTopics(self):
        self.main_logger.info("ROS topics are waiting.")
        if self.simulation_enebled:
            # rospy.wait_for_message('/simulation_uav_state', FullState)

            rospy.Subscriber("/simulation_uav_state",FullState,self.uav_simulation_callback)
            rospy.Subscriber("/simulation_uav_state",FullState,self.rate.callback_hz,callback_args="/simulation_uav_state")
            self.main_logger.info("Subscribed 'simulation_uav_state' topic.")


        if self.real_enebled:
            # rospy.wait_for_message('/real_uav_state', FullState)
            rospy.Subscriber("/real_uav_state",FullState,self.uav_real_callback)
            rospy.Subscriber("/real_uav_state",FullState,self.rate.callback_hz,callback_args="/real_uav_state")
            self.main_logger.info("Subscribed 'real_uav_state' topic.")


        # rospy.wait_for_message('/uav_commands', FullCommand)
        rospy.Subscriber("/uav_commands",FullCommand,self.uav_command_callback)
        rospy.Subscriber("/uav_commands",FullCommand,self.rate.callback_hz,callback_args="/uav_commands")
        self.main_logger.info("Subscribed 'uav_commands' topic.")

        rospy.Subscriber("/uav_trajectory_commands",FullTrajectoryCommand,self.uav_traj_command_callback)
        rospy.Subscriber("/uav_trajectory_commands",FullTrajectoryCommand,self.rate.callback_hz,callback_args="/uav_trajectory_commands")
        self.main_logger.info("Subscribed 'uav_commands' topic.")



    def createLoggers(self,parent_folder,logger_id,vehicle_numbers):
        log_folder_path = os.path.join(parent_folder,logger_id)
        print(os)
        print(log_folder_path)
        os.makedirs(log_folder_path)
        self.main_logger = self.setup_logger("main",os.path.join(log_folder_path,"main.log"))
        self.topics_logger = self.setup_logger("topics",os.path.join(log_folder_path,"topics.log"))
        self.main_logger.info("logger 'main' created succesfully")
        self.main_logger.info("creating {} uav and {} ugv loggers".format(*vehicle_numbers))
        uav_folder_path = os.path.join(log_folder_path,"uav")
        os.makedirs(uav_folder_path)
        for i in range(vehicle_numbers[0]):
            self.uav_loggers["uav_{}".format(i+1)] = self.setup_logger("uav_{}".format(i+1),os.path.join(uav_folder_path,"uav_{}.log".format(i+1)))
        ugv_folder_path = os.path.join(log_folder_path,"ugv")
        os.makedirs(ugv_folder_path)
        for i in range(vehicle_numbers[1]):
            self.ugv_loggers["ugv_{}".format(i+1)] = self.setup_logger("ugv_{}".format(i+1),os.path.join(ugv_folder_path,"uav_{}.log".format(i+1)))
            

        self.main_logger.info("all loggers have been created")

        
    def setup_logger(self,name, log_file, level=logging.INFO):
        """To setup as many loggers as you want"""

        handler = logging.FileHandler(log_file)        
        handler.setFormatter(self.formatter)

        logger = logging.getLogger(name)
        logger.setLevel(level)
        logger.addHandler(handler)
        
        msg = "logger '" + name + "' created succesfully."
        if not name == "main":
            logger.info(msg)
            logger.info("not any information yet")
        return logger
    
    def uav_simulation_callback(self,data):
        for state in data.fullState:
            msg_array = [[state.id.replace("/","_")],
                         [state.active],
                         [[state.pose.position.x,state.pose.position.y,state.pose.position.z],
                          [state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w]],
                         [[state.twist.linear.x,state.twist.linear.y,state.twist.linear.z],
                          [state.twist.angular.x,state.twist.angular.y,state.twist.angular.z]],
                         [state.accel.x,state.accel.y,state.accel.z]]
            # self.uav_sim_state_data[state.id.replace("/","_")] = msg_array
            self.uav_loggers[state.id.replace("/","_")].info("SIM_STATE : " + str(msg_array))

            # self.uav_loggers[state.id.replace("/","_")].info(msg_array)

    def uav_command_callback(self,data):
        for command in data.allcommands:
            msg_array = [[command.id.replace("/","_")],
                         [command.active],
                         [[command.twist.linear.x,command.twist.linear.y,command.twist.linear.z],
                          [command.twist.angular.x,command.twist.angular.y,command.twist.angular.z]]]
            # self.uav_command_data[command.id.replace("/","_")] = msg_array
            self.uav_loggers[command.id.replace("/","_")].info("COMMAND : " + str(msg_array))


    def uav_traj_command_callback(self,data):
        for command in data.allcommands:
            msg_array = [[command.id.replace("/","_")],
                         [command.active],
                         [[command.pose.position.x,command.pose.position.y,command.pose.position.z],
                          [command.pose.orientation.x,command.pose.orientation.y,command.pose.orientation.z,command.pose.orientation.w]],
                         [[command.twist.linear.x,command.twist.linear.y,command.twist.linear.z],
                          [command.twist.angular.x,command.twist.angular.y,command.twist.angular.z]],
                         [command.accelaration.x,command.accelaration.y,command.accelaration.z],
                         [command.yaw]]
            # self.uav_traj_command_data[command.id.replace("/","_")] = msg_array
            self.uav_loggers[command.id.replace("/","_")].info("T_COMMAND : " + str(msg_array))

    def uav_real_callback(self,data):
        for state in data.fullState:
            msg_array = [[state.id.replace("/","_")],
                         [state.active],
                         [[state.pose.position.x,state.pose.position.y,state.pose.position.z],
                          [state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w]],
                         [[state.twist.linear.x,state.twist.linear.y,state.twist.linear.z],
                          [state.twist.angular.x,state.twist.angular.y,state.twist.angular.z]],
                         [state.accel.x,state.accel.y,state.accel.z]]
            # self.uav_real_state_data[state.id.replace("/","_")] = msg_array
            self.uav_loggers[state.id.replace("/","_")].info("REAL_STATE : " + str(msg_array))


    def log_states(self):
        while not rospy.is_shutdown():
            print("logging states")
            print(self.uav_sim_state_data)
            for data in self.uav_sim_state_data:
                self.uav_loggers[data].info("SIM_STATE : " + str(self.uav_sim_state_data[data]))
            for data in self.uav_real_state_data:
                self.uav_loggers[data].info("REAL_STATE : " + str(self.uav_real_state_data[data]))

    def log_commands(self):
        while not rospy.is_shutdown():
            for data in self.uav_command_data:
                self.uav_loggers[data].info("COMMAND : " + str(self.uav_command_data[data]))
            for data in self.uav_traj_command_data:
                self.uav_loggers[data].info("T_COMMAND : " + str(self.uav_traj_command_data[data]))

    def log_rates(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.simulation_enebled:
                try:
                    self.topics_logger.info("/simulation_uav_state " + str(self.rate.get_hz("/simulation_uav_state")[0]))
                except:
                    pass
        
            if self.real_enebled:
                try:
                    self.topics_logger.info("/real_uav_state " + str(self.rate.get_hz("/real_uav_state")[0]))
                except:
                    pass
            try:
                self.topics_logger.info('/uav_commands ' + str(self.rate.get_hz('/uav_commands')[0]))
            except:
                pass
            try:
                self.topics_logger.info('/uav_trajectory_commands ' + str(self.rate.get_hz('/uav_trajectory_commands')[0]))
            except:
                pass
        self.ending_time = time.time()
        self.flight_time = self.ending_time - self.flight_starting_time
        msg = "flight duration: " + str(self.flight_time)
        self.main_logger.info(msg)

    def log_all(self):
        self.process1.start()
        self.process2.start()
        self.process3.start()

if __name__ == "__main__":
    logger = GammaLogger()
    print("starting logger"*100)
    while not rospy.is_shutdown():
        rospy.spin()
        


