ﺏ
# MinSnap-CrazyFlie2.1   
 The 2022 Gamma Team Integrated Swarm System Repository Reduced In Scope for 1 Drone Navigation Flight. It is open to development and its usability has been confirmed in swarm studies.


## References
<a id="1">[1]</a> 
EKLENECEK.

<a id="2">[2]</a> 
EKLENECEK.

<a id="3">[3]</a> 
EKLENECEK.

# Setup 
Firs create folder with name:
```
cd
mkdir catkin_ws
```

Download the package with git clone or from IDE.

```
cd catkin_ws
git clone https://github.com/imamim/MinSnap-CrazyFlie2.1.git
```

Then run the `catkin_make` command at main directory.

```
cd GammaSwarm
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
cd catkin_ws/GammaSwarm
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
source /home/Your_Computer_username/catkin_ws/crazyswarm/ros_ws/devel/setup.bash
source /home/Your_Computer_username/catkin_ws/GammaSwarm/devel/setup.bash
```

Configure all of the dependencies of PyBullet
# Dependencies
```
pip3 install shapely
pip3 install rtree
pip3 install plotly
```

## Starting System
For V1.0 of system, you can use this 1 line code (After Setup Process you can do this):

```
roslaunch GammaSwarm allRun.launch
```

If you want configure of mission parameter please refer this 2 code which that given relative path!

```
src/GammaSwarm/src/GammaSwarm/mainSystem/Parameters.py
src/GammaSwarm/src/GammaSwarm/simulationSystems/SimulationParameter.py
```

## Developing
Bu kısım Türkçe anlatılacaktır!

GammaSwarm System V1.0 için düzenlenebilinecek şeyler şunlardır:
1- Mission.py içerisindeki modeList değişkeni:
```
src/GammaSwarm/src/GammaSwarm/mainSystem/Mission.py
```
2- Görev fonksiyonlarının yerine getirilmesi için izlenmesi gereken aşamalar şunlardır!

-Modes.py içerisine modu çalıştıracak yeni bir fonksiyon eklenmelidir. Nasıl bir yapıya sahip olacağı 'takeOff' fonksiyonunda görülmektedir.
-Ardından bu fonksiyon içerisinde kullanılacak merkezcil ve dağıtık fonksiyonlar 'uavClass.py' ve 'MerkezcilClass.py' içerisine eklenmelidir.

## PyBullet 
https://github.com/utiasDSL/gym-pybullet-drones 

## PIP 
https://pip.pypa.io/en/stable/ 

# General Use
--> Use launch files to configure standardized packages 

--> Do not use any recursive or try-expect function 

--> After every Ros dependent change (msg,service vs.), do the catkin_make at main dir

--> Before every push, make sure that you did pull 


# General Structure
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
