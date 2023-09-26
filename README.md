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

You will find a simple example of creating a mission under this heading. For detailed information about where each parameter is used, please review the wiki. (Or create discussion)

Let the requirements of the task we want to do be as follows. Let's say we have 4 aircraft. We want to lift these vehicles to a height of 1 meter and form them into a two-dimensional V-shaped formation in the air. After this formation is taken, it is desired to navigate to a certain point while preserving the formation. After completing these tasks, the aircraft land.

_Open ```src/GammaSwarm/src/GammaSwarm/mainSystem/mission.py ```_

###### NOTE

It is very important for the flight order that the swarm forms a formation before each navigation. Navigation without entering the formation is not recommended. If you want the aircraft to navigate on its own, please see the [Individual Navigation](https://github.com/imamim/minSnap-CrazyFlie2.1) resource **:heavy_exclamation_mark:**

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

- Initialize mode will already be in the main source code. This mode should always be at the top of the list and only its parameters should be edited.

- If we want to make our flight only in a simulation environment

  (```simulation_enabled = True``` , ```real_enabled = False```)
  
- If we want to make our flight only in its real application

  (```simulation_enabled = False``` , ```real_enabled = True```)
  
  **:heavy_exclamation_mark:** The situation where both real and simulation environments work simultaneously has not been implemented. 
  
- Let's have 4 aircraft (```number_of_agent = 4```)
  
- The starting formation must remain common.

  (```starting_formation = FORMATIONTYPES.common```)
  
  **:heavy_exclamation_mark:** Area dimensions does not matter! It is taken only as input to be used in other tasks. 
  
- Let's add Takeoff mode to the list
  
  ```{MISSIONMODES.take_off : TakeoffParams(takeoff_height = 0.7 ,threshold = 0.08)} , ```
  
- We want all vehicles to take off 1 meter above the ground.

  (```takeoff_height = 1.0```)
  
- A loiter should be added between each mode. Loiter time can be adjusted to the desired value as long as it is not too small.

  ```{MISSIONMODES.loiter : LoiterParams(loiter_time = 3)} ,```
  
- Now the takeoff is over and we can get into formation.

  ```{MISSIONMODES.formation2D : FormationParams2D(formation_type = FORMATIONTYPES.v,each_distance = 0.65,corner_count = 4,threshold=0.07)} ,```
  
- Some parameters need to be adjusted so that our formation type, the distance between each aircraft (distance between corners or points, not diagonal distance) and the number of corners are equal to the number of vehicles. You can check out the wiki for detailed explanations.

  (```formation_type = FORMATIONTYPES.v``` , ```each_distance = 0.65``` )
  
- A certain amount of loiter is added.

  ```{MISSIONMODES.loiter : LoiterParams(loiter_time = 3)} ,```
  
- Navigation can now begin after the vehicles become stable. Desired destination:

  - **:round_pushpin:** [x = 1 meter , y = 1 meter , z = 1 meter] -> ```navigation_waypoints = [Position(1,1,1)]```

  - The maximum speed of the swarm center during navigation will be 1 [m/s].
  
  - The formation type should be the same as the formation we are in (```FORMATIONTYPES.v```) and the distance between them should remain the same (```each_distance = 0.65```): 
  
  - **:one:** ```{MISSIONMODES.navigation : NavigationParams(agressiveness_kt = 30 ,max_velocity = 1, navigation_waypoints = [Position(1,1,1)], threshold = 0.08)},```
  
  - **:two:** ```{MISSIONMODES.formation2D : FormationParams2D(formation_type = FORMATIONTYPES.v,each_distance = 0.65,corner_count = 4,threshold=0.07)},```
  
  - **:three:** ```{MISSIONMODES.loiter : LoiterParams(loiter_time = 3)},```

- Then, finally, landing should be done.

  ```MISSIONMODES.landing : LandingParams(threshold = 0.07)},```
  
- The following task should also be added so that the system understands that the task is completed.

  ```{MISSIONMODES.completed : True}```
  
**:warning:** It is not recommended to change the threshold values in all tasks unless you are familiar with the subject. All of them were fixed to a suitable value after experimental tests. Most of the time you will not encounter any errors.


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
