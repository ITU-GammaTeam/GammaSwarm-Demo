#Rahman ve Rahim Olan Allah'ın Adıyla,Hamd O'na Mahsustur!
#Muvaffakiyetimiz Yalnızca O'na aittir!
from gammaSwarm import GammaSwarm
from MerkezcilClass import *
from UavClass import *
from Initializer import *
from copy import deepcopy
import rospy
import numpy as np


class Modes:

    def __init__(self,modeList):
        self.modeList = modeList
        self.modeListIndex = 0
        self.mode = list(modeList[self.modeListIndex].keys())[0]

        init_params = modeList[self.modeListIndex].get(self.mode)

        self.real_enabled = init_params.real_enabled 
        initial_position = systemInitializer(init_params)

        #Haberleşme yapısı oluşturuldu,sistem kuruldu

        self.Swarm = GammaSwarm(init_params,initial_position)
        #merkezcil classlar oluşturuldu
        self.central_formation = FormationClass()
        self.trajectory_class = TrajGenerator()

        self.area_dimension = init_params.area_dimensions

        #Mode changed
        self.modeListIndex+=1
        self.mode = list(modeList[self.modeListIndex].keys())[0]
        


    def takeOffStep(self,takeoff_params):
        completed_list = []
        self.Swarm.update_SwarmCenter() #Update swarm center using last data. Also we have synchronized using mutex!

        #Controller or another think computation for drones. This is also means virtual computer on drones. (But we are not, we have only 1 base PC)
        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.takeOffPID(uav.initial_position,takeoff_params.takeoff_height,self.Swarm.swarm_center,takeoff_params.threshold)
            else :
                uav_result = True

            completed_list.append(uav_result)

            #Example of another algoirthm use for individual uavs.
            uav.collisionAvoidance()

            #For ROS generate Command Message in Class and after publish commands using this generated data! 
            """
            If there was a companion computer on each drone, each drone would publish its own location or command to autopilot. For the situation we have, 
            this structure gives better results.
            """
            uav.generateCommandMessage()
        #END OF COMPUTATION FOR EACH DRONE LOOP

        self.Swarm.publishCommand()
        
        #Change Mode when all drones complete works, (inactive drones always return true)
        if all(completed_list) == True:
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0]        
        del completed_list




    def loiterStep(self,loiter_params):
        completed_list = []
        self.Swarm.update_SwarmCenter() #Update swarm center using last data. Also we have synchronized using mutex!
        
        #Controller or another think computation for drones. This is also means virtual computer on drones. (But we are not, we have only 1 base PC)
        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.loiter(loiter_params.loiter_time,self.Swarm.swarm_center)
            else :
                uav_result = True #Bundan dolayı sıkıntı oluyor!!!! Activation Flag Mevzusu

            completed_list.append(uav_result)

            #Example of another algoirthm use for individual uavs.
            uav.collisionAvoidance()

            #For ROS generate Command Message in Class and after publish commands using this generated data! 
            """
            If there was a companion computer on each drone, each drone would publish its own location or command to autopilot. For the situation we have, 
            this structure gives better results.
            """
            uav.generateCommandMessage()
        #END OF COMPUTATION FOR EACH DRONE LOOP

        self.Swarm.publishCommand()

        #Change Mode when all drones complete works, (inactive drones always return true)
        if all(completed_list) == True:
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0]
            for uav in self.Swarm.uav_list:
                uav.passing_loiter_time = 0 
                uav.loiter_active = False   
        del completed_list



    def formationStep2D(self,formation_params):
        completed_list = []
        self.Swarm.update_SwarmCenter() #Update swarm center using last data. Also we have synchronized using mutex!

        # If the formation has not been created when we come to this mode, calculations are made once and then the operations are continued using these values. 
        # This flag is reset at the end of the mode
        if self.central_formation.formation_created == False:
            self.central_formation.meanAnglePointCalculator(self.Swarm.uav_count,self.Swarm.uav_list,self.Swarm.swarm_center)
            self.central_formation.generateFormationPoints(self.Swarm.swarm_center,self.Swarm.uav_count,formation_params)
            self.central_formation.FairMacar(self.Swarm.uav_list,self.Swarm.uav_count)
            self.central_formation.formation_created = True
    
        #Controller or another think computation for drones. This is also means virtual computer on drones. (But we are not, we have only 1 base PC)
        i = 0
        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.goToPID(Position(self.central_formation.formation_points[self.central_formation.goal_indexes[i]][0],self.central_formation.formation_points[self.central_formation.goal_indexes[i]][1],self.central_formation.formation_points[self.central_formation.goal_indexes[i]][2]),formation_params.threshold,self.Swarm.swarm_center)
                i+=1
            else :
                uav_result = True
                i+=1

            completed_list.append(uav_result)

            #Example of another algoirthm use for individual uavs.
            uav.collisionAvoidance()
            
            #For ROS generate Command Message in Class and after publish commands using this generated data! 
            """
            If there was a companion computer on each drone, each drone would publish its own location or command to autopilot. For the situation we have, 
            this structure gives better results.
            """
            uav.generateCommandMessage()
        #END OF COMPUTATION FOR EACH DRONE LOOP

        self.Swarm.publishCommand()
        
        #Change Mode when all drones complete works, (inactive drones always return true)
        if all(completed_list) == True:
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0] 
            self.central_formation.formation_created = False
            #TODO TUM PARAMETRELER SIFIRLANACK MERKEZCILCLASSTAKI        
        del completed_list




    def simpleNavigationStep(self,navigation_params):
        completed_list = []
        self.Swarm.update_SwarmCenter() #Update swarm center using last data. Also we have synchronized using mutex!

        #Bismillahirrahmanirrahim.
        # If the navigation trajectory (for swarm center) has not been created when we come to this mode, 
        # calculations are made once and then the operations are continued using these values. 
        # This flag is reset at the end of the mode
        if self.trajectory_class.trajectory_generated == False:
            """
            Generate trajectory for initial_position,navigation waypoints and trajectory parameters. This functions works for nearly all case. 
            This is simple structure.Gives data to function, an internally generate trajectory. Trajectory data is stored inside. 
            After generation we need to initialize executer.
            
            The Executor structure works by measuring time simultaneously. That's why it's sensitive.
            
            All trajectory executers should be deleted after the job is finished. 
            Even if it is not deleted, the data inside must be reset so that it is ready for the new trajectory.
            """
            self.trajectory_class.generate_trajectory(self.Swarm.swarm_center,navigation_params.navigation_waypoints,navigation_params.max_velocity,navigation_params.agressiveness_kt)
            self.trajectory_class.trajectory_generated = True
            self.trajectory_executer = TrajectoryExecuter(des_state = self.trajectory_class.get_des_state,Tmax = self.trajectory_class.TS[-1])

        #Bismillahirrahmanirrahim

        #Calculate time taken inside class and return desired trajectory state. If trajectory done return last state.
        desired_traj_state,traj_completed = self.trajectory_executer.step() 

        #Kontrol Stateleri:
        #1- Time has exceeded Tmax
        #2- Distance from the swarm center to the target point
        #3- Or checking whether each drone has reached the relative position it should be in.

        #We have mode switching between with respect to traj_completed parameter. If finish use PID to reach last position, if not use trajectory commands.
        for uav in self.Swarm.uav_list:
            #Execute Trajectory
            if uav.activation_flag == True and traj_completed == False:
                #Update Desired Setpoint
                uav.navigationEqualizer(desired_traj_state.desired_position,desired_traj_state.desired_velocity,desired_traj_state.desired_acceleration,desired_traj_state.desired_yaw,desired_traj_state.desired_omega)
                #Calculate actual error
                uav_result = uav.calculateNavigationError(self.trajectory_class.waypoints[-1],self.Swarm.swarm_center,navigation_params.threshold) #After navigation, if norm<Thrsehold uav_result become True
                #Not used #Example of another algoirthm use for individual uavs.
                uav.collisionAvoidance()
                #Then update message!
                uav.generateTrajectoryCommandMessage()
            #Execute PID
            elif uav.activation_flag == True and traj_completed == True:
                uav.navigationEqualizer(desired_traj_state.desired_position,desired_traj_state.desired_velocity,desired_traj_state.desired_acceleration,desired_traj_state.desired_yaw,desired_traj_state.desired_omega)
                uav_result = uav.goToPID(uav.desired_trajectory_position,navigation_params.threshold,self.Swarm.swarm_center)
                uav.collisionAvoidance()
                uav.generateCommandMessage()

            else :
                uav_result = True

            completed_list.append(uav_result)
        #END OF COMPUTATION FOR EACH DRONE LOOP

        if traj_completed == False:
            self.Swarm.publishTrajectoryCommand()
        elif traj_completed == True:
            self.Swarm.publishCommand()
        
        #Change Mode when all drones complete works, (inactive drones always return true)
        if all(completed_list) == True: 
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0] 
            self.trajectory_class.trajectory_generated = False
            del self.trajectory_executer #Reset or delete trajectory
        del completed_list


    def landingStep(self,landing_params):
        completed_list = []
        self.Swarm.update_SwarmCenter() #Update swarm center using last data. Also we have synchronized using mutex!

        #Controller or another think computation for drones. This is also means virtual computer on drones. (But we are not, we have only 1 base PC)
        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.landingPID(uav.initial_position,landing_params.threshold)
                #print(uav_result,uav.id,"target height: ",takeoff_height,"Current height: ",uav.current_position.x,uav.current_position.y,uav.current_position.z)
            else :
                uav_result = True
            completed_list.append(uav_result)
            #Example of another algoirthm use for individual uavs.
            uav.collisionAvoidance()
            #For ROS generate Command Message in Class and after publish commands using this generated data! 
            """
            If there was a companion computer on each drone, each drone would publish its own location or command to autopilot. For the situation we have, 
            this structure gives better results.
            """
            uav.generateCommandMessage()
        #END OF COMPUTATION FOR EACH DRONE LOOP

        self.Swarm.publishCommand()
        
        if all(completed_list) == True:
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0]         
        del completed_list




############################################# NOT USED IN SWARM MISSION THIS USING IN INDIVIDUAL NAVIGATION MISSION IN ANOTHER REPO #######################

    """""
    @Author: Muhammed Emin Hamamcı - github:imamim --> https://github.com/imamim/minSnap-CrazyFlie2.1
    """
    #TODO If usage neede, pls go in the mission.py ---> change simpleNavigationStep() ---->individualNavigationStep()
    #Pls use only 1 drone when flight! Elhamdulillah navigation cozuldu
    #This function for 1 Drones TESTING THE TRAJECTORY
    def individualNavigationStep(self,navigation_params):
        completed_list = []
        self.Swarm.update_SwarmCenter()

        #Bismillahirrahmanirrahim. Bizim buradaki amacımız navigation ile hedefe yaklaşabileceğimiz kadar yaklaşmak. Ardından eger hala varamadıysak.GoTo ile 
        #ihtiyac duyulan threshold degerine dronelar sokulur. GoTo mod olarak veyahut burada cagırılabılır! 
        #Mode baslangıcında trajectory uretilir ve bir daha buraya girmez!
        if self.trajectory_class.trajectory_generated == False:
            #Hızlandırmalıyız bu generation'u
            self.trajectory_class.generate_trajectory(self.Swarm.uav_list[0].current_position,navigation_params.navigation_waypoints,navigation_params.max_velocity,navigation_params.agressiveness_kt)
            self.trajectory_class.trajectory_generated = True
            #TODO Trajectory Executer olusturuldugunda ilk stebi donse ve sonrasında steplense iyi olabilir ? veyahut timedan ayarlayacagım!
            self.trajectory_executer = TrajectoryExecuter(des_state = self.trajectory_class.get_des_state,Tmax = self.trajectory_class.TS[-1])
            #Simdi bunu kurduk her trajectory olusturulunca tekrar kurulacak ve steplenecek

        #Bismillahirrahmanirrahim
        desired_traj_state,traj_completed = self.trajectory_executer.step() #t = 0,0+1/56,0+2/56,......
        #Kontrol Stateleri:
        #1- Time'ın Tmax'ı geçmiş olması
        #2- Sürü merkezinin hedef noktasına olan uzaklığı
        #3- Veya her drone için

        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                #Update Desired Setpoint
                uav.individualNavigationEqualizer(desired_traj_state)
                #Calculate actual error
                uav_result = uav.individualCalculateNavigationError(self.trajectory_class.waypoints[-1],self.Swarm.swarm_center,navigation_params.threshold) #After navigation, if norm<Thrsehold uav_result become True
                #Not used #Example of another algoirthm use for individual uavs.
                uav.collisionAvoidance()
                #Then update message!
                uav.generateTrajectoryCommandMessage()
            else :
                uav_result = True

            completed_list.append(uav_result)

        self.Swarm.publishTrajectoryCommand()
        
        if traj_completed == True: #and: #traj_completed == True:
            print("BITTTTTTIIIII",self.Swarm.uav_list[0].current_position.x,self.Swarm.uav_list[0].current_position.y,self.Swarm.uav_list[0].current_position.z)
            self.Swarm.uav_list[0].initial_position = self.Swarm.uav_list[0].current_position
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0] 
            self.trajectory_class.trajectory_generated = False
            del self.trajectory_executer
            #TODO TUM PARAMETRELER SIFIRLANACK MERKEZCILCLASSTAKI        
        del completed_list

    