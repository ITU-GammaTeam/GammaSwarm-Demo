ﺏ
<img src="https://github.com/ITU-GammaTeam/GammaSwarm-Demo/blob/main/logo.png" width="720">


# GammaSwarm-Demo 

The 2022 Gamma Team Integrated Swarm System Repository reduced in scope for any number of drones for actions:

- Takeoff 
- Loiter
- 2D Formation 
- Navigation Flight
- Landing

with _CrazyFlie 2.1_ drones. It is open to development and its usability has been confirmed in swarm studies.For more information please look at wiki of this project.


**Or you can look Teknofest 2023 Heterogeneous Swarm Robots Competition _[Project Report](https://drive.google.com/file/d/1Wdwlqqi9w5jY0oGLyqyqK5ytAj6Xoze8/view?usp=sharing)_ 
at :two:nd place team:heavy_exclamation_mark:**

## Installation

### Dependencies

This repository was developed on Ubuntu 20.04 and all tests were performed in this environment with ROS Noetic. The system is built on the ROS Noetic version. It is recommended to use the following configurations:
```
Ubuntu 20.04
ROS Noetic :
http://wiki.ros.org/noetic/Installation/Ubuntu
```
And other libraries:
```
pip3 install shapely
pip3 install rtree
pip3 install plotly
```

### Setup 
Firs create folder with name:
```
cd
mkdir catkin_ws
```

Download the package with git clone or from IDE.

```
cd catkin_ws
git clone https://github.com/ITU-GammaTeam/GammaSwarm-Demo.git
```

Then run the `catkin_make` command at main directory.

```
cd GammaSwarm-Demo
catkin_make
```

After make, install pybullet 
```
cd src/GammaSwarm/src/
pip3 install -e .
```

Go the main directory catkin_ws and do `catkin_make`
```
cd 
cd catkin_ws/GammaSwarm-Demo
catkin_make
```

Lastly add your setup.bash file your system .bashrc
```
gedit ~/.bashrc
```

Then if you don't source ROS do this:
```
source /opt/ros/noetic/setup.bash
```

And add this two line you last line of .bashrc file:
```
source /home/Your_Computer_username/catkin_ws/GammaSwarm-Demo/devel/setup.bash
```

After that close current terminal and open new terminal.

## Starting System

<img src="https://github.com/ITU-GammaTeam/GammaSwarm-Demo/blob/main/example.gif" width="680">

For V1.0 of system, you can use this 1 line code (After Setup Process you can do this):

```
roslaunch GammaSwarm allRun.launch
```

If you want configure of mission parameter please refer this code:

```
src/GammaSwarm/src/GammaSwarm/mainSystem/mission.py
```

For more information about all parameters please refer this 2 code which that given relative path!

```
src/GammaSwarm/src/GammaSwarm/mainSystem/Parameters.py
src/GammaSwarm/src/GammaSwarm/simulationSystems/SimulationParameter.py
```

### Example

Bu başlık altında basit bir görev oluşturma örneği bulacaksınız. Her parametrenin nerede kullanıldığı ile alakalı ayrıntılı bilgi için lütfen wiki'yi inceleyiniz.
Yapmak istediğimiz görevin isterleri şunlar olsun. Elimizde bulunan 4 adet hava aracını 1 metre yüksekliğe kaldırıp havada iki boyutlu V şeklinde bir formasyon almasını istiyoruz. Bu formasyon alındıktan sonra formasyonu koruyarak belli 1 noktaya navigasyon yapılması istenmektedir. Bu görevleri tamamladıktan sonra ise hava araçlarının inişi gerçekleştirilir. 

_Open ```src/GammaSwarm/src/GammaSwarm/mainSystem/mission.py ```_

###### NOTE

Her navigasyon öncesi sürünün bir formasyon şeklinde bulunması uçuş düzeni açısından çok önemlidir, formasyona girilmeden sürü halinde navigasyon yapılması tavsiye edilmez. Hava aracının tek başına navigasyon yapmasını istiyorsanız lütfen [Individual Navigation](https://github.com/imamim/minSnap-CrazyFlie2.1) kaynağına bakınız **:heavy_exclamation_mark:**


#### Mode List

```
modeList = [
            #TODO Simulation does not work with 1 agent, this mainSystem can work 1 drone but we have problem at simulation. Can't render drone.
            #For information of parameters pls look at Parameters.py
            {MISSIONMODES.initialize  :      InitializerParams(number_of_agent = 4,simulation_enabled = True,real_enabled = False, starting_formation = FORMATIONTYPES.common, area_dimension = [(-1.6, 1.6), (-1.9, 1.9), (0, 1.5)])},
            
            {MISSIONMODES.take_off    :      TakeoffParams(takeoff_height = 1.0 ,threshold = 0.08)} , 
            {MISSIONMODES.loiter      :      LoiterParams(loiter_time = 3)} ,

            {MISSIONMODES.formation2D :      FormationParams2D(formation_type = FORMATIONTYPES.v,each_distance = 0.65,corner_count = 4,threshold=0.07)},
            {MISSIONMODES.loiter      :      LoiterParams(loiter_time = 3)},
            
            {MISSIONMODES.navigation  :      NavigationParams(agressiveness_kt = 30 ,max_velocity = 1, navigation_waypoints = [Position(1,1,1)], threshold = 0.08)},
            {MISSIONMODES.formation2D :      FormationParams2D(formation_type = FORMATIONTYPES.v,each_distance = 0.65,corner_count = 4,threshold=0.07)},
            {MISSIONMODES.loiter      :      LoiterParams(loiter_time = 3)},
        
            {MISSIONMODES.landing     :      LandingParams(threshold = 0.07)},

            {MISSIONMODES.completed   :      True}
            
            ]
```

#### Walkthrough without GUI for V1:

- Initialize modu zaten başlangıçta kodun içerisinde olacaktır. Bu mode her zaman listenin başında olmalı ve sadece paremetreleri düzenlenmelidir.

- Uçuşumuzu sadece simülasyon ortamında yapmak istiyorsak (```simulation_enabled = True``` , ```real_enabled = False``` )
  
- Uçuşumuzu sadece gerçek ortamında yapmak istiyorsak (```simulation_enabled = False``` , ```real_enabled = True``` )
  
- Hem gerçek hem simülasyon ortamının aynı anda çalıştığı durum implemente edilmemiştir **:heavy_exclamation_mark:**
  
- 4 tane hava aracımız bulunsun (```number_of_agent = 4```)
  
- Başlangıç formasyonunun common olarak kalmalıdır. (```starting_formation = FORMATIONTYPES.common```)
  
- Alan ölçülerinin bir önemi yoktur! Başka görevlerde kullanılmak üzere sadece input olarak alınmaktadır. **:heavy_exclamation_mark:**
  
- Takeoff modunu listeye ekleyelim: ```{MISSIONMODES.take_off : TakeoffParams(takeoff_height = 0.7 ,threshold = 0.08)} , ```
  
- Araçların hepsinin 1 metre yukarıya kalkmasını istiyoruz. (```takeoff_height = 1.0```)
  
- Her mod arasına bir loiter eklenmelidir. Loiter zamanı çok fazla küçük olmadığı sürece istenilen değere ayarlanılabilir. ```{MISSIONMODES.loiter : LoiterParams(loiter_time = 3)} ,```
  
- Artık kalkış bitti ve formasyona girebiliriz.  ```{MISSIONMODES.formation2D : FormationParams2D(formation_type = FORMATIONTYPES.v,each_distance = 0.65,corner_count = 4,threshold=0.07)} ,```
  
- Formasyon tipimizi, her hava aracı arasında bulunması istenen mesafe (köşeler veya noktalar arası mesafe, köşegen mesafesi değil) ve köşe sayısı araç sayısına eşit olacak şekilde ayarlamalar yapılmalıdır. Ayrıntılı açıklamalar için wiki'ye göz atabilirsiniz. (```formation_type = FORMATIONTYPES.v``` , ```each_distance = 0.65``` )
  
- Belli miktarda loiter eklenir. ```{MISSIONMODES.loiter : LoiterParams(loiter_time = 3)} ,```
  
- Artık araçlar stabil duruma geçtikten sonra navigasyon hareketine başlanılabilir. Gidilmesi istenen nokta

  **:round_pushpin:** [x = 1 meter , y = 1 meter , z = 1 meter] -> ```navigation_waypoints = [Position(1,1,1)]```

  Navigasyon sırasında sürü merkezinin maksimum hızı 1 [m/s] olacaktır.
  Formasyon çeşidi de hangi formasyonda isek o olmalıdır (```FORMATIONTYPES.v```) ve aralarındaki mesafe aynı kalmalıdır (```each_distance = 0.65```): 
  
  **:one:** ```{MISSIONMODES.navigation : NavigationParams(agressiveness_kt = 30 ,max_velocity = 1, navigation_waypoints = [Position(1,1,1)], threshold = 0.08)},```
  
  **:two:** ```{MISSIONMODES.formation2D : FormationParams2D(formation_type = FORMATIONTYPES.v,each_distance = 0.65,corner_count = 4,threshold=0.07)},```
  
  **:three:** ```{MISSIONMODES.loiter : LoiterParams(loiter_time = 3)},```

- Ardından son olarak landing yapılmalıdır. ```MISSIONMODES.landing : LandingParams(threshold = 0.07)},```
  
- Görevin bittiğinin sistem tarafından anlaşılması için şu görev de eklenmelidir. ```{MISSIONMODES.completed : True}```
  
Bütün görevlerdeki threshold değerlerinin konuya hakim değilseniz değiştirilmesi tavsiye edilmez. Hepsi deneysel testlerden sonrası uygun bir değere sabitlenmiştir. Büyük oranda bir hata ile karşılaşmayacaksınızdır.


## Developing



## References
<a id="1">[1]</a> 
EKLENECEK.

<a id="2">[2]</a> 
EKLENECEK.

<a id="3">[3]</a> 
EKLENECEK.

## PyBullet 
https://github.com/utiasDSL/gym-pybullet-drones 

## PIP 
https://pip.pypa.io/en/stable/ 

### General Use
--> Use launch files to configure standardized packages 

--> Do not use any recursive or try-expect function 

--> After every Ros dependent change (msg,service vs.), do the catkin_make at main dir

--> Before every push, make sure that you did pull 


### General Structure
```bash
Gamma Swarm
├── CMakeLists.txt
├── launch
│   └── allRun.launch
├── msg
│   ├── example.msg
│   ├── FullCommand.msg
│   ├── FullState.msg
│   ├── FullTrajectoryCommand.msg
│   ├── TrajectoryCommand.msg
│   ├── UavCommand.msg
│   └── UavState.msg
├── package.xml
├── runSystem.sh
├── setup.py
├── src
│   ├── CITATION.cff
│   ├── GammaSwarm
│   │   ├── logging
│   │   │   ├── classes.py
│   │   │   ├── gammaAnalyser.py
│   │   │   ├── gammaLogger.py
│   │   │   └── logo.png
│   │   └── mainSystem
│   │   │   ├── enums.py
│   │   │   ├── gammaSwarm.py
│   │   │   ├── Initializer.py
│   │   │   ├── initializer_util.py
│   │   │   ├── MerkezcilClass.py
│   │   │   ├── mission.py
│   │   │   ├── Modes.py
│   │   │   ├── Parameters.py
│   │   │   ├── State.py
│   │   │   ├── UavClass.py
│   │   │   └── utils.py
│   │   └── simulationSystems
│   │   │   ├── ControllerUtils.py
│   │   │   ├── environment.py
│   │   │   ├── obstacle.urdf
│   │   │   ├── simulation_executer.py
│   │   │   └── SimulationParameter.py
│   │   └── realSystems
│   │   │   ├── cfSwarm.py
│   │   │   ├── realExecuter.py
│   │   │   └── realManager.py
│   ├── LICENSE
│   ├── README.md
│   └── setup.py
└── srv
    ├── RealServiceMessage.srv
    └── ServiceMessage.srv
```
