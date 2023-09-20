import sys
import os

cwd =os.path.dirname(os.path.realpath(__file__))
slashes = [n for (n,e) in enumerate(cwd) if e=='/'] 
relName = cwd[:slashes[3]] 
mainsysTarget = "/minSnap-CrazyFlie2.1/src/GammaSwarm/src/GammaSwarm/mainSystem"
sys.path.append(relName + mainsysTarget)



import pybullet as p
import cflib.crtp
from State import * 
import rospy 
import time 
from GammaSwarm.msg import UavState, FullState, FullTrajectoryCommand
from GammaSwarm.srv import RealServiceMessage, RealServiceMessageResponse
from gammaSwarm import * 
import numpy as np
from Parameters import *
from cfSwarm import *



#TODO baslangicta ucustan once estimator reset edilmeli, Light check yaptırılmalı Initializer'da time.sleep atılıp biraz beklenmeli

class RealManager:
    def __init__(self):
        global relName #TODO Reponun Relative Position cekilir Kodun en üst tarafına bakın
        self.yamlName = "/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml" #TODO Bunun değişmesi gerekiyor. Bunun içerisinde drone bilgileri vardı. CFlib için URI bilgileri bir dosyaya kaydedilip çekilmeli
                                                                                   #TODO Buraya bu dosyanın path'i girilmelidir.
        self.yamlPath = relName + self.yamlName
        self.uris = [
                     "radio://0/125/2M/E7E7E7E7C1",
                     #"radio://0/125/2M/E7E7E7E7C2",
                     "radio://0/125/2M/E7E7E7E7C3",
                     #"radio://1/115/2M/E7E7E7E7C4",
                     "radio://1/115/2M/E7E7E7E7C9"
                     ] #TODO degisecek
        #TODO ID eslestirmesi olacak

        self.args = {}
        self.trajectory_command = None
        self.command = None
        self.server_manual_shutdown = False
        
        ## bu değişilecek ##### #TODO ??
        self.fullstate_flag = False
        self.trajectory_flag = False
        #######################
        ## in gui, this will be checked insAllah####
        self.active = True 
        self.allActive = False                             #TODO Aktif bir hale getirilmeli.Mustafa da bu işe dahil olmalı
        ############################################
        
    
        # maybe we need to subscribe to the main system
        rospy.init_node("Real-System")

        self.server()
        
        self.publisher = rospy.Publisher("real_uav_state", FullState, queue_size = 10)
        self.realLocationPublisher()

        rospy.wait_for_message("uav_commands", FullCommand) 
        rospy.Subscriber("uav_trajectory_commands", FullTrajectoryCommand, self.trajectory_call_back)
        rospy.Subscriber("uav_commands", FullCommand, self.call_back)

    def initializeReal(self,request):        
        if request.ClientRequest == True:
            ## Initialize CFLIB BASED Swarm Class                    
            cflib.crtp.init_drivers()
            factory = CachedCfFactory(rw_cache='./cache')
            
            #TODO kod buraya kadar çalışıyor Swarm class içerisinde error alıyor
            self.swarm = Swarm(self.uris, factory=factory)  

            self.drone_num = len(self.swarm._cfs) #TODO Bunun ile alakalı not alındı. Bağlanabildiğimiz kadarını bildirmeliyiz. Bağlanamıyorsak ignore etmeliyiz.
            #TODO GENEL BIR ID LISTEMIZ OLSA ONUN ICINDE GEZERIZ
            
            #TODO Reset Estimators and doLightCheck ----> Problem olursa Yoruma alınmalı
            #self.swarm.lightCheck() ----> Problem cikardi
            """
            Reset estimator buga sebep oldu koyulmasına gerek yok
            """
            #self.swarm.reset_estimators() #TODO Yorumdan çıkarılmalı
            time.sleep(10)

            positions,orientations = self.getAllPositions() 
            full_message = RealServiceMessageResponse()
            uav_count = 0
            for position,orientation in zip(positions,orientations):
                msg = UavState()
                ## 
                msg.id = "uav/" + str(uav_count + 1)
                msg.active = True
                msg.pose.position.x = position[0]
                msg.pose.position.y = position[1]
                msg.pose.position.z = position[2]
                quaternion = p.getQuaternionFromEuler([orientation[0], orientation[1], orientation[2]])

                msg.pose.orientation.x = quaternion[0]
                msg.pose.orientation.y = quaternion[1]
                msg.pose.orientation.z = quaternion[2]
                msg.pose.orientation.w = quaternion[3]
                # no need to the twist message
                
                uav_count+=1
                # may be a mistake here like
                full_message.FullStateList.append(msg)
        
        self.server_manual_shutdown = True
        ## buraya RealServiceMessageResponse da gelebilir 
        return full_message

    ## server is going to be here 
    ## however this server will respond to the request sent from gammaSwarm or 
    ## initializer in order to send the initial locations of drones
    ## service message will be like bool 
    ## respond will be UavState for all drones I think :D 
    
    def trajectory_call_back(self, data):
        self.fullstate_flag = False
        self.trajectory_flag = True
        self.trajectory_command = data.allcommands
    
    def call_back(self, data):
        self.trajectory_flag = False
        self.fullstate_flag = True
        self.command = data.allcommands


    def getAllActivationStatus(self):
        """
        Alttaki getAllPositions gibi self.swarm.get_estimated_activations() fonksiyonunun çağırarak
        Activation dictionary'i buraya çekmelidir. Sonrasında veri işlenerek liste haline sokulup realLocationPublisherda cagırılmalıdır.
        """
        swarm_activation = self.swarm.get_estimated_activations() #Swarm activation ismi kafa karıştırmasın. Şuan tüm swarmun activasyonuna değil individual activasyon bakılıyo
        activation_status = []
        for uri in swarm_activation.keys():
            active = swarm_activation[uri]
            activation_status.append(active)

        return activation_status
        


    def getAllPositions(self):
        swarm_position, swarm_orientation = self.swarm.get_estimated_positions()
        #TODO ID ESLESTIRMESI GEREKEBILIR
        positions = []
        orientations = []
        if len(swarm_position) == len(swarm_orientation):
            for uri in swarm_position.keys():
                pose = np.array([swarm_position[uri].x, swarm_position[uri].y, swarm_position[uri].z])        
                orient = np.array([swarm_orientation[uri].roll, swarm_orientation[uri].pitch, swarm_orientation[uri].yaw])
                positions.append(pose)
                orientations.append(orient)
            return positions,orientations
        else:
            print("HATA VAR") #TODO Drone verileri duzgun gelmiyor demektir.
    

    # def getAllPositions(self): #TODO Initialize edilirken bu fonksiyon kullanılarak konumlar çekilmektedir. Ya yeni bir yapı kurulmalı ya da sabit bir yapı olarak buraya CFLIB ile entegre edilmelidir. Swarm classı nasıl olacak ?
    #     positions = []
    #     orientations = []
    #     for i in range(self.drone_num):
    #         drone_position = self.swarm.allcfs.crazyflies[i].position()
    #         positions.append(drone_position)
    #     return positions
        
        
    def server(self):
        r = rospy.Rate(60)

        self.service = rospy.Service("Real-Initializer-Server", RealServiceMessage, self.initializeReal) 
        while not self.server_manual_shutdown:
            r.sleep()
        
        self.service.shutdown()
        del self.service
           
            
    # sends locations of drones to the main system
    def realLocationPublisher(self):
        #positions = self.swarm.allcfs.crazyflies
        #positions = random_loc_generator(5)
        positions,orientations = self.getAllPositions()
        activation_status = self.getAllActivationStatus()
        full_message = FullState()
        uav_count = 0
        for position,orientation,active in zip(positions,orientations,activation_status):
            msg = UavState()
            msg.id = "uav/"+str(uav_count+1)
            msg.active = active
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            quaternion = p.getQuaternionFromEuler([orientation[0], orientation[1], orientation[2]])

            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]
            # no need to the twist message
            uav_count+=1
            # may be a mistake here like
            full_message.fullState.append(msg)
            
        self.publisher.publish(full_message)



    def fromCflibActivationCheck(self):
        """
        Returns a True positive activation state dict keyed by uris
        """
        _all_linkstatus = self.swarm.get_all_link_status()
        _all_visibility_status = self.swarm.get_all_bs_visibility()
        _all_acc_values = self.swarm.get_all_acc_values()


        for uri in self.swarm._unshifted_dict.keys():
            if self.swarm._unshifted_dict[uri] == True:
                if _all_linkstatus[uri] == False:
                    self.swarm._kill_counter[uri] -= 3
                elif _all_visibility_status[uri] == False:
                    self.swarm._kill_counter[uri] -= 2
                elif _all_acc_values[uri] > 1:
                    self.swarm._kill_counter[uri] -= 5 
                else:
                    self.swarm._kill_counter[uri] = 65

        #True initlenmiş unshifted sözlüğü sadece False olarak güncellenebilir
        for uri in self.swarm._unshifted_dict.keys():
            if self.swarm._kill_counter[uri] <= 0:
                self.swarm._unshifted_dict[uri] = False

        #print("kill_counter: ", self.swarm._kill_counter)
        #print("unshifted_dict: ",self.swarm._unshifted_dict)

#Rahman ve Rahim Olan Allah'ın Adıyla, Hamd Olsun Allah'a

    #TODO Active mevzusu ile LAND anlaşılıyordu bunun eklenmesi gerekiyor

    def updateSwarmSetPoint(self): #TODO executer'da cagirilacak fonksiyon
        arguments = self.generateArgsDict()
        func = self.swarm.applyCommand
        self.swarm.parallel(func, args_dict = arguments)
                           #TODO Args dict olusturtulup parallel safe cagirtilacak

    def generateArgsDict(self): #TODO Gelen bilgiler uri'lere gore duzenlenerek aktarılacak. ID eslesmesi sıkıntısının yasanmaması gerekiyor.
        
        if self.fullstate_flag == True:
            self.args = {}
            for command,uri in zip(self.command,self.uris):
                if uri not in self.args:
                    self.args[uri]  = [np.array([command.twist.linear.x,command.twist.linear.y,command.twist.linear.z,0])]
                self.allActive = self.allActive or command.active
        
        
        if self.trajectory_flag == True:
            self.args = {}
            for command,uri in zip(self.trajectory_command,self.uris):
                if uri not in self.args:
                    self.args[uri]  = [np.array([command.twist.linear.x,command.twist.linear.y,command.twist.linear.z,command.twist.angular.z])]
                self.allActive = self.allActive or command.active
        return self.args

        #TODO Document'teki gibi args dict olusturacak urilere gore

    def fromMainActivationCheck(self):
        #Active Flag'i False olan bir drone var ise bunun URI'si ile cfSwarm'daki URI ile eslesen scf icin disarmCrazyFlie fonksiyonu cagirilir.
        for command,uri in zip(self.command,self.uris):
            if command.active == False:
                cf = self.swarm._cfs[uri]
                self.swarm.disarmCrazyFlie(cf)
    
    def nonActiveEmergencyStop(self):
        """
        Ustteki fonksiyon gibi active flag'ı false olan drone'u kapatmalı ve link'i close etmeli.
        disarmCrazyFlie fonksiyonu verilen cf objesini kullanarak drone'u kapatıyor ve bağlantıyı kesiyor.
        """
        
        swarm_activation = self.swarm.get_estimated_activations() #Swarm activation ismi kafa karıştırmasın. Şuan tüm swarmun activasyonuna değil individual activasyon bakılıyo
        for uri in swarm_activation.keys():
            if swarm_activation[uri] == False:
                cf = self.swarm._cfs[uri]
                self.swarm.disarmCrazyFlie(cf)
        
