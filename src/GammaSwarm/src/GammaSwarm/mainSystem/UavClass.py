#RAHMAN VE RAHİM OLAN ALLAH'ın ADIYLA!!!! Hamd yalnızca O'na mahsustur!!!
#Elhamdulillah

"""
Bu Class dağıtık olarak çalışacak işlemleri içerekcektir. Modlar işler iken buradaki fonksiyonlar ile beraber Merkezcil olarak kullanılacak
fonksiyonları içeren sınıf kullanılacaktır.
"""
import rospy
import numpy as np
from State import *
from GammaSwarm.msg import UavCommand, TrajectoryCommand
import math
import time
from copy import deepcopy

class UavClass:

    def __init__(self,id,initial_position,inital_orientation):
        """Constructor.
            Args:
                id (int): Integer ID in range [0, 255]. The last byte of the robot's
                    radio address.
                initialPosition (iterable of float): Initial position of the robot:
                    [x, y, z]. Typically on the floor of the experiment space with
                    z == 0.0.
                tf (tf.TransformListener): ROS TransformListener used to query the
                    robot's current state. -----> ?
        
        
        Publisher'lar burada oluşturulmalı
        
        """
        self.id = id
        self.activation_flag = True #TODO bu aktif hale gelmeli
        self.current_velocity = Velocity(0,0,0,0,0,0)
        self.initial_position = Position(initial_position[0],initial_position[1],initial_position[2],inital_orientation[0],inital_orientation[1],inital_orientation[2])
        self.current_position = Position(initial_position[0],initial_position[1],initial_position[2],inital_orientation[0],inital_orientation[1],inital_orientation[2])
        self.old_position =  Position(initial_position[0],initial_position[1],initial_position[2],inital_orientation[0],inital_orientation[1],inital_orientation[2])  
        self.desired_velocity = Velocity(0,0,0,0,0,0)

        #TODO !!!!!gereksiz degiskenler duzenlenmeli,yani baglantili (kullanıldıgı) oldugu yontemler duzenlenmeli
        self.desired_trajectory_position = Position(0,0,0)
        self.desired_trajectory_velocity = Velocity(0,0,0,0,0,0)
        self.desired_trajectory_acceleration = Velocity(0,0,0,0,0,0)
        self.desired_trajectory_yaw = 0
        self.desired_trajectory_omega = Velocity(0,0,0,0,0,0)
        
        
        self.command_message = UavCommand()
        self.trajectory_command_message = TrajectoryCommand()
        #LOITER DEGISKENLERI
        self.passing_loiter_time = 0 
        self.loiter_position = None
        self.loiter_active = False
        self.loiter_old_position = None
        self.old_omega = np.array([0,0,0])
        self.formation_position = Position(0,0,0)
        self.position_wrt_center = Position(0,0,0) #TODO initialize edilmeli, swarm center'a göre yerdeki
        self.theoretical_position_wrt_center = Position(0,0,0)
        

    def takeOffPID(self,takeoff_start_position, hedef_yukseklik, swarm_center, threshold,duration = None): #TODO Start position+hedef yukseklik
        temp_hedef_yukseklik = hedef_yukseklik
        manipulated_hedef_yukseklik = hedef_yukseklik
        takeoff_completed = False
        distance_dz = 0
        distance_iz = 0
        distance_dx = 0
        distance_ix = 0
        distance_dy = 0
        distance_iy = 0
        Kp = 0.06  
        Kd = 0.03
        Ki = 0.53
        Kp_z = 0.13
        Kd_z = 0.25
        Ki_z = 0.27
        distancez = manipulated_hedef_yukseklik - self.current_position.z
        distancex = takeoff_start_position.x - self.current_position.x
        distancey = takeoff_start_position.y - self.current_position.y

        distance_dz = max(min(distancez - distance_dz, 8),-8)
        distance_dx = max(min(distancex - distance_dx, 8),-8)
        distance_dy = max(min(distancey - distance_dy, 8),-8)

        distance_iz = max(min(distancez + distance_iz, 8),-8)
        distance_ix = max(min(distancex + distance_ix, 8),-8)
        distance_iy = max(min(distancey + distance_iy, 8),-8)


        pid_z = Kp_z*distancez + Kd_z*distance_dz + Ki_z*distance_iz
        pid_x = Kp*distancex + Kd*distance_dx + Ki*distance_ix
        pid_y = Kp*distancey + Kd*distance_dy + Ki*distance_iy

        self.desired_velocity.z = pid_z 
        self.desired_velocity.x = pid_x
        self.desired_velocity.y = pid_y
        #print(self.desired_velocity.x , self.desired_velocity.y, self.desired_velocity.z)
        self.old_position.z = self.current_position.z
        self.old_position.x = self.current_position.x
        self.old_position.y = self.current_position.y

        norm = math.sqrt(distancex**2+distancey**2+(temp_hedef_yukseklik-self.current_position.z)**2)
        # print("ID:",self.id,"X:",distancex,"Y:",distancey,"Z:",(temp_hedef_yukseklik-self.current_position.z))
        print("ID: ",self.id , " ERROR: ",norm,"\n")
        if norm<threshold:
            takeoff_completed = True
            takeoff_res_position = Position(takeoff_start_position.x,takeoff_start_position.y,hedef_yukseklik,0,0,0)
            self.position_wrt_center = Position(self.current_position.x - swarm_center.x, self.current_position.y-swarm_center.y,self.current_position.z-swarm_center.z)
            self.initial_position = takeoff_res_position

        return takeoff_completed

    def goToPID(self,hedef_position,threshold,swarm_center):
        goTo_completed = False
        distance_dz = 0
        distance_iz = 0
        distance_dx = 0
        distance_ix = 0
        distance_dy = 0
        distance_iy = 0

        Kp = 0.4  # 0.50
        Kd = 0.01562
        Ki = 0.103
        Kp_z = 0.22
        Kd_z = 0.20
        Ki_z = 0.27

        distancez = hedef_position.z - self.current_position.z
        distancex = hedef_position.x - self.current_position.x
        distancey = hedef_position.y - self.current_position.y

        distance_dz = max(min(distancez - distance_dz, 8), -8)
        distance_dx = max(min(distancex - distance_dx, 8), -8)
        distance_dy = max(min(distancey - distance_dy, 8), -8)

        distance_iz = max(min(distancez + distance_iz, 8), -8)
        distance_ix = max(min(distancex + distance_ix, 8), -8)
        distance_iy = max(min(distancey + distance_iy, 8), -8)

        pid_z = Kp_z*distancez + Kd_z*distance_dz + Ki_z*distance_iz
        pid_x = Kp*distancex + Kd*distance_dx + Ki*distance_ix
        pid_y = Kp*distancey + Kd*distance_dy + Ki*distance_iy

        self.desired_velocity.z = pid_z 
        self.desired_velocity.x = pid_x
        self.desired_velocity.y = pid_y

        distance_dx = distancex
        distance_dy = distancey
        distance_dz = distancez

        distance2d = math.sqrt(distancex**2 + distancey**2)

        norm = math.sqrt((hedef_position.x - self.current_position.x)**2+(hedef_position.y - self.current_position.y)**2+(hedef_position.z - self.current_position.z)**2)
        #print("ERROR:",norm)
        # print("ID:",self.id,"X:",distancex,"Y:",distancey,"Z:",distancez,"\n", "VELOCITY",self.desired_velocity.x , self.desired_velocity.y , self.desired_velocity.z)
        #format_float = "{:.2f}".format(norm)
        print("ID: ",self.id , "  ERROR: ",norm,"\n")

        if norm < threshold:
            goTo_completed = True
            #TODO aslında burada hata var, ama bunu neglect edebiliriz. Cunku threshold degeri icindeyizdir demek oluyor bu! ---> Duzeltilebilir.
            #TODO BURAYA BAK
            self.formation_position = hedef_position
            self.position_wrt_center = Position(self.current_position.x - swarm_center.x, self.current_position.y-swarm_center.y,self.current_position.z-swarm_center.z)
            self.initial_position = hedef_position
       
        return goTo_completed


    def landingPID(self,takeoff_start_position,threshold):
        landing_completed = False
        distance_dz = 0
        distance_iz = 0
        distance_dx = 0
        distance_ix = 0
        distance_dy = 0
        distance_iy = 0
        Kp = 0.06  
        Kd = 0.03
        Ki = 0.53
        distancez = 0 - self.current_position.z
        distancex = takeoff_start_position.x - self.current_position.x
        distancey = takeoff_start_position.y - self.current_position.y

        distance_dz = max(min(distancez - self.old_position.z, 8),-8)
        distance_dx = max(min(distancex - self.old_position.x, 8),-8)
        distance_dy = max(min(distancey - self.old_position.y, 8),-8)

        distance_iz = max(min(distancez + distance_iz, 8),-8)
        distance_ix = max(min(distancex + distance_ix, 8),-8)
        distance_iy = max(min(distancey + distance_iy, 8),-8)


        pid_z = Kp*distancez + Kd*distance_dz + Ki*distance_iz
        pid_x = Kp*distancex + Kd*distance_dx + Ki*distance_ix
        pid_y = Kp*distancey + Kd*distance_dy + Ki*distance_iy

        self.desired_velocity.z = pid_z
        self.desired_velocity.x = pid_x
        self.desired_velocity.y = pid_y
        self.old_position.z = self.current_position.z
        self.old_position.x = self.current_position.x
        self.old_position.y = self.current_position.y

        norm = math.sqrt(distancex**2+distancey**2+distancez**2)
        print("Land2Ground:  ",norm)
        if norm<threshold:
            landing_completed = True
            self.initial_position = self.current_position
            self.desired_velocity.x = 0
            self.desired_velocity.y = 0
            self.desired_velocity.z = 0
            self.desired_velocity.wx = 0
            self.desired_velocity.wy = 0
            self.desired_velocity.wz = 0
            self.activation_flag = False

        return landing_completed

    def calculateLandingNorm(self, hedef_position, threshold):
        landing_flag = False
        norm = math.sqrt((hedef_position.x - self.current_position.x)**2+(hedef_position.y - self.current_position.y)**2+(0 - self.current_position.z)**2)
        if norm < threshold:
            landing_flag = True
            self.desired_velocity.x = 0
            self.desired_velocity.y = 0
            self.desired_velocity.z = 0
            self.desired_velocity.wx = 0
            self.desired_velocity.wy = 0
            self.desired_velocity.wz = 0
        return landing_flag


    def navigationEqualizer(self,traj_pos,traj_vel,traj_acc,traj_yaw,traj_omega):
        "Calculate Trajectory Output Without controller"
        r_diff = np.array([self.position_wrt_center.x,self.position_wrt_center.y,self.position_wrt_center.z]) #Hep sabit bu position_wrt_center ? Traj boyunca
        velocity = traj_vel + np.cross(traj_omega,r_diff)
        alpha = (traj_omega-self.old_omega)*60
        self.old_omega = traj_omega.copy()
        acceleration = traj_acc + np.cross(alpha,r_diff) - (np.linalg.norm(traj_omega)**2)*r_diff
        position = r_diff + traj_pos
        
        self.desired_trajectory_position.x = position[0]
        self.desired_trajectory_position.y = position[1]
        self.desired_trajectory_position.z = position[2]

        self.desired_trajectory_velocity.x = velocity[0]
        self.desired_trajectory_velocity.y = velocity[1]
        self.desired_trajectory_velocity.z = velocity[2]
        
        self.desired_trajectory_acceleration.x = acceleration[0]
        self.desired_trajectory_acceleration.y = acceleration[1]
        self.desired_trajectory_acceleration.z = acceleration[2]

        self.desired_trajectory_yaw = traj_yaw

        self.desired_trajectory_omega.x = traj_omega[0]
        self.desired_trajectory_omega.y = traj_omega[1]
        self.desired_trajectory_omega.z = traj_omega[2]


    def individualNavigationEqualizer(self,desired_traj_state): #TODO Karısılıklık olmamasi icin fonksiyon ismi degistirilebilir!
        self.desired_trajectory_position.x = desired_traj_state.desired_position[0]
        self.desired_trajectory_position.y = desired_traj_state.desired_position[1]
        self.desired_trajectory_position.z = desired_traj_state.desired_position[2]

        self.desired_trajectory_velocity.x = desired_traj_state.desired_velocity[0]
        self.desired_trajectory_velocity.y = desired_traj_state.desired_velocity[1]
        self.desired_trajectory_velocity.z = desired_traj_state.desired_velocity[2]
        
        self.desired_trajectory_acceleration.x = desired_traj_state.desired_acceleration[0]
        self.desired_trajectory_acceleration.y = desired_traj_state.desired_acceleration[1]
        self.desired_trajectory_acceleration.z = desired_traj_state.desired_acceleration[2]

        self.desired_trajectory_yaw = desired_traj_state.desired_yaw

        self.desired_trajectory_omega.x = desired_traj_state.desired_omega[0]
        self.desired_trajectory_omega.y = desired_traj_state.desired_omega[1]
        self.desired_trajectory_omega.z = desired_traj_state.desired_omega[2]


    def calculateNavigationError(self,last_point,swarm_center,threshold):
        "Formasyon halindeki hesap"
        navigation_completed = False
        r_diff = np.array([self.position_wrt_center.x,self.position_wrt_center.y,self.position_wrt_center.z])
        #last point swarm center'in olması gereken pointti
        desired_last_pose = last_point+r_diff #Last position!
        #print(desired_last_pose)
        diff = np.array([self.current_position.x-desired_last_pose[0],self.current_position.y-desired_last_pose[1],self.current_position.z-desired_last_pose[2]])
        norm = np.linalg.norm(diff)
        #print("navigation_norm",norm)
        print("ID: ",self.id , "Navigation Norm: ",norm,"\n")
        if norm<threshold:
            navigation_completed = True
            self.initial_position = Position(desired_last_pose[0], desired_last_pose[1],desired_last_pose[2]) #Thats okey for other controller. PID mod bitsede son position goturur.
            self.position_wrt_center = Position(desired_last_pose[0] - swarm_center.x, desired_last_pose[1]-swarm_center.y,desired_last_pose[2]-swarm_center.z)

        return navigation_completed
    
    
    def individualCalculateNavigationError(self,last_point,swarm_center,threshold):
        "Individual hesap"
        navigation_completed = False
        #last point swarm center'in olması gereken pointti
        desired_last_pose = last_point
        #print(desired_last_pose)
        diff = np.array([self.current_position.x-desired_last_pose[0],self.current_position.y-desired_last_pose[1],self.current_position.z-desired_last_pose[2]])
        norm = np.linalg.norm(diff)
        print("navigation_norm",norm)
        if norm<threshold:
            navigation_completed = True
            self.initial_position = Position(desired_last_pose[0], desired_last_pose[1],desired_last_pose[2])
            self.position_wrt_center = Position(desired_last_pose[0] - swarm_center.x, desired_last_pose[1]-swarm_center.y,desired_last_pose[2]-swarm_center.z)
        return navigation_completed



    def loiter(self,duration,swarm_center):
        if self.loiter_active == False:
            self.loiter_loop_start_time = time.time()
            #print("INITIAL POSE ALINDI")
            self.loiter_position = self.initial_position
            self.loiter_old_position = self.initial_position
            self.loiter_active = True
            
        elif self.loiter_active == True:
            loiter_completed = False
            #hedef_yukseklik = self.loiter_position.z + ((5-self.loiter_position.z)/10)*(max((self.loiter_position.z/3),0.5))
            hedef_yukseklik = self.loiter_position.z
            distance_dz = 0
            distance_iz = 0
            distance_dx = 0
            distance_ix = 0
            distance_dy = 0
            distance_iy = 0
            Kp = 0.06
            Kd = 0.0015
            Ki = 0.53

            distancez = hedef_yukseklik - self.current_position.z
            distancex = self.loiter_position.x - self.current_position.x
            distancey = self.loiter_position.y - self.current_position.y

            distance_dz = max(min(distancez - self.old_position.z, 8),-8)
            distance_dx = max(min(distancex - self.old_position.x, 8),-8)
            distance_dy = max(min(distancey - self.old_position.y, 8),-8)
            distance_iz = max(min(distancez + distance_iz, 8),-8)
            distance_ix = max(min(distancex + distance_ix, 8),-8)
            distance_iy = max(min(distancey + distance_iy, 8),-8)

            pid_z = Kp*distancez + Kd*distance_dz + Ki*distance_iz
            pid_x = Kp*distancex + Kd*distance_dx + Ki*distance_ix
            pid_y = Kp*distancey + Kd*distance_dy + Ki*distance_iy

            self.desired_velocity.z = pid_z
            self.desired_velocity.x = pid_x
            self.desired_velocity.y = pid_y

            self.old_position.z = distancez
            self.old_position.x = distancex
            self.old_position.y = distancey
            self.passing_loiter_time = time.time() - self.loiter_loop_start_time

            norm = math.sqrt((self.loiter_position.x - self.current_position.x)**2+(self.loiter_position.y - self.current_position.y)**2+(self.loiter_position.z - self.current_position.z)**2)
            format_float = "{:.2f}".format(self.passing_loiter_time)
            print("ID: ",self.id , "  ERROR: ",norm," Passed Time: ",format_float,"\n")

            #print("PASSED TIME",self.passing_loiter_time,)
            if self.passing_loiter_time>=duration:
                loiter_completed = True
                #I think this is must be current position.
                self.position_wrt_center = Position(self.current_position.x - swarm_center.x, self.current_position.y-swarm_center.y,self.current_position.z-swarm_center.z)
                self.initial_position = deepcopy(self.loiter_position)#deepcopy(self.current_position)#deepcopy(self.loiter_position) #Eger ki istenen pozisyona ulasmadı ise errora sebep olur
            return loiter_completed


    #Example of another algoirthm use for individual uavs.
    def collisionAvoidance(self):
        output = []
        return output


    def generateCommandMessage(self):
        msg = UavCommand()
        msg.id = self.id
        msg.active = self.activation_flag
        msg.twist.linear.x = self.desired_velocity.x
        msg.twist.linear.y = self.desired_velocity.y
        msg.twist.linear.z = self.desired_velocity.z
        msg.twist.angular.x = self.desired_velocity.wx
        msg.twist.angular.y = self.desired_velocity.wy
        msg.twist.angular.z = self.desired_velocity.wz
        self.command_message = msg

    def generateTrajectoryCommandMessage(self):
        msg = TrajectoryCommand()
        msg.id = self.id
        msg.active = self.activation_flag
        msg.pose.position.x = self.desired_trajectory_position.x
        msg.pose.position.y = self.desired_trajectory_position.y
        msg.pose.position.z = self.desired_trajectory_position.z
        
        msg.twist.linear.x = self.desired_trajectory_velocity.x
        msg.twist.linear.y = self.desired_trajectory_velocity.y
        msg.twist.linear.z = self.desired_trajectory_velocity.z
        
        msg.twist.angular.x = self.desired_trajectory_omega.x
        msg.twist.angular.y = self.desired_trajectory_omega.y
        msg.twist.angular.z = self.desired_trajectory_omega.z

        msg.accelaration.x = self.desired_trajectory_acceleration.x
        msg.accelaration.y = self.desired_trajectory_acceleration.y
        msg.accelaration.z = self.desired_trajectory_acceleration.z

        msg.yaw = self.desired_trajectory_yaw
        
        self.trajectory_command_message = msg

    def getGeneratedMessageToTrajectory(self):
        msg = TrajectoryCommand()
        msg.id = self.id
        msg.active = self.activation_flag
        msg.pose.position.x = self.desired_trajectory_position.x
        msg.pose.position.y = self.desired_trajectory_position.y
        msg.pose.position.z = self.desired_trajectory_position.z
        
        msg.twist.linear.x = self.desired_velocity.x
        msg.twist.linear.y = self.desired_velocity.y
        msg.twist.linear.z = self.desired_velocity.z
        
        msg.twist.angular.x = self.desired_velocity.wx
        msg.twist.angular.y = self.desired_velocity.wx
        msg.twist.angular.z = self.desired_velocity.wx

        msg.accelaration.x = self.desired_trajectory_acceleration.x
        msg.accelaration.y = self.desired_trajectory_acceleration.y
        msg.accelaration.z = self.desired_trajectory_acceleration.z
        self.trajectory_command_message = msg
    

    def getStateCallback(self,data,args):
        #For adding arguments look at : https://answers.ros.org/question/231492/passing-arguments-to-callback-in-python/
        #Subscriber calıstıgında
        self.current_velocity.x = data.vel.x
        self.current_position.x = data.pos.x #gibi
        rospy.loginfo(data)








                                                        # YEDEK - backup functions
####################################################################################################################################################################
                                                    #NOT GUARATNEED TO WORK!!!!!!!!!!!!!!!!!!



    #TODO Not a Good Solution. Initial Position vs. Problem
    def loiterNoTime(self,swarm_center):
        #self.loiter_loop_start_time = time.time()
        #print("INITIAL POSE ALINDI")
        self.loiter_position = self.initial_position
        self.loiter_old_position = self.initial_position
        
    
        #loiter_completed = False
        hedef_yukseklik = self.loiter_position.z
        distance_dz = 0
        distance_iz = 0
        distance_dx = 0
        distance_ix = 0
        distance_dy = 0
        distance_iy = 0
        Kp = 0.06
        Kd = 0.001
        Ki = 0.53
        distancez = hedef_yukseklik - self.current_position.z
        distancex = self.loiter_position.x - self.current_position.x
        distancey = self.loiter_position.y - self.current_position.y
        distance_dz = max(min(distancez - self.old_position.z, 8),-8)
        distance_dx = max(min(distancex - self.old_position.x, 8),-8)
        distance_dy = max(min(distancey - self.old_position.y, 8),-8)
        distance_iz = max(min(distancez + distance_iz, 8),-8)
        distance_ix = max(min(distancex + distance_ix, 8),-8)
        distance_iy = max(min(distancey + distance_iy, 8),-8)
        pid_z = Kp*distancez + Kd*distance_dz + Ki*distance_iz
        pid_x = Kp*distancex + Kd*distance_dx + Ki*distance_ix
        pid_y = Kp*distancey + Kd*distance_dy + Ki*distance_iy
        self.desired_velocity.z = pid_z
        self.desired_velocity.x = pid_x
        self.desired_velocity.y = pid_y
        #print(self.desired_velocity.x , self.desired_velocity.y, self.desired_velocity.z)
        self.old_position.z = distancez
        self.old_position.x = distancex
        self.old_position.y = distancey
        norm = np.linalg.norm(np.array([self.current_position.x - self.loiter_position.x,self.current_position.y - self.loiter_position.y,self.current_position.z - self.loiter_position.z]))
        print("LOITER NORM :",norm) #GERCEKTE NASIL BIR STABILLIGE SAHIP KRITIK


    def loiterForWait(self,duration,swarm_center):
        if self.loiter_active == False:
            self.loiter_loop_start_time = time.time()
            #print("INITIAL POSE ALINDI")
            self.loiter_position = self.current_position
            self.loiter_old_position = self.current_position
            self.loiter_active = True
            
        elif self.loiter_active == True:
            loiter_completed = False
            #hedef_yukseklik = self.loiter_position.z + ((5-self.loiter_position.z)/10)*(max((self.loiter_position.z/3),0.5))
            hedef_yukseklik = self.loiter_position.z
            distance_dz = 0
            distance_iz = 0
            distance_dx = 0
            distance_ix = 0
            distance_dy = 0
            distance_iy = 0
            Kp = 0.06
            Kd = 0.001
            Ki = 0.53
            distancez = hedef_yukseklik - self.current_position.z
            distancex = self.loiter_position.x - self.current_position.x
            distancey = self.loiter_position.y - self.current_position.y
            distance_dz = max(min(distancez - self.old_position.z, 8),-8)
            distance_dx = max(min(distancex - self.old_position.x, 8),-8)
            distance_dy = max(min(distancey - self.old_position.y, 8),-8)
            distance_iz = max(min(distancez + distance_iz, 8),-8)
            distance_ix = max(min(distancex + distance_ix, 8),-8)
            distance_iy = max(min(distancey + distance_iy, 8),-8)
            pid_z = Kp*distancez + Kd*distance_dz + Ki*distance_iz
            pid_x = Kp*distancex + Kd*distance_dx + Ki*distance_ix
            pid_y = Kp*distancey + Kd*distance_dy + Ki*distance_iy
            self.desired_velocity.z = pid_z
            self.desired_velocity.x = pid_x
            self.desired_velocity.y = pid_y
            #print(self.desired_velocity.x , self.desired_velocity.y, self.desired_velocity.z)
            self.old_position.z = distancez
            self.old_position.x = distancex
            self.old_position.y = distancey
            self.passing_loiter_time = time.time() - self.loiter_loop_start_time
            #print(self.passing_loiter_time)
            norm = np.linalg.norm(np.array([self.current_position.x - self.loiter_position.x,self.current_position.y - self.loiter_position.y,self.current_position.z - self.loiter_position.z]))
            print("LOITER NORM :",norm) #GERCEKTE NASIL BIR STABILLIGE SAHIP KRITIK

            print("PASSED TIME",self.passing_loiter_time)
            if self.passing_loiter_time>=duration:
                loiter_completed = True
                self.position_wrt_center = Position(self.loiter_position.x - swarm_center.x, self.loiter_position.y-swarm_center.y,self.loiter_position.z-swarm_center.z)
                self.initial_position = deepcopy(self.loiter_position)#deepcopy(self.current_position)#deepcopy(self.loiter_position)
            return loiter_completed

