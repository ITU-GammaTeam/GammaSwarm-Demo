#!/usr/bin/env python3

from realManager import *
import rospy
#Rahman ve Rahim Olan Allah'ın Adıyla
# gerçek sistem
#   ilk olarak önemli olan noktalardan bir tanesi ilk drone konumlarının yarışma içerisinde alınması lazım.
#   daha sonra gerçek konumların anlık olarak main system e iletilmesi lazım. 
#   main system görevlere göre ve droneların anlık konumlarına göre hedef konumları hesaplaması lazım. 
#   daha sonra cs sisteminin bu sisteme abone olup gitme talimatını dronelara vermesi lazım


# sistem entegre ise
# yine konumların gerçek veya reelden gelmesi lazım
# ana sistemden gelen komutların burada uygulanması lazım veya şu anda kalsın

#TODO Kapama fonksiyonu eklenecek
            
def main():
    #cmd_msg = rc.command
    real_manager = RealManager() 
    freqSetter = rospy.Rate(15)
    
    while not rospy.is_shutdown():
        if (real_manager.trajectory_command is None) and (real_manager.command is None):
            print("None Message alınmıyor")
            continue
        #TODO Ikı trajectory commanda gerek var mı ?
        #TODO land yapıldıgı anlasılıp motor realManager'da kapattırılmalı
        #if rc.command[i].active == False:
        #            print("LAND YAPILDI")
        #            cf.cmdStop()
        real_manager.allActive = False
        
        #Do the update state (give command)
        real_manager.updateSwarmSetPoint()
        
        #If the active flag == Flase stopping the motor. For corresponding drone not all!
        real_manager.fromMainActivationCheck()


        #Validate drones availabilty states
        real_manager.fromCflibActivationCheck()
        real_manager.nonActiveEmergencyStop() #--> içine close links de koyulmalı!!!

        #Publish location data
        real_manager.realLocationPublisher()
        
        #TODO is bitince kod sıkıntıya girince vs. finally veya if ile diasbleSwarm deyip close_links çalıştırılmalı. Normal ve normal olmayan durum için

        #timeHelper.sleep(0.0001)
        freqSetter.sleep()
        if not real_manager.allActive:
            print("broken the loop")
            break


if __name__ == "__main__":
    main()
