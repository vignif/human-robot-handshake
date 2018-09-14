# iit changes
To run you must:
open a terminal (Ctrl+Alt+T) and write:
```
roscore
```
open another terminal:
```
roscd qb_interface/conf/
rosparam load config.yaml
rosrun qb_interface qb_force_control
rostopic pub /qb_class/hand_ref qb_interface/handRef "closure: [9000]"
``` 
where the grasp closure is within [0-19000]
Change stiffness with 
```rosparam set /stiffness 0.1``` 
where stiffness is from [0-1.0]



# qb_interface_node
ROS node for communicate with sequence and multi-sequence of qbDevice

# Install
1. Create a new folder in your catkin workspace call "qb_interface"
2. Load all file in it, no other files are request

Otherwise:
  - .deb [indigo, jade] if you request it on our website

###### Note: This code has been developed for ROS Indigo on ubuntu 14.04. No warranty for other distributions.

# Configure
Main parameters can be configurable by a YAML file, which is request during the call to qb_interface_node with "roslaunch" method.
If you do not specify your yaml own custom file, by variable yamlFile:="PathToYourConfFile", a default yaml fill will be set.

example

**communication port**

port: '/dev/ttyUSB0'

**[Equilibrium/Preset] -> True , [P1/P2/PL] -> False**

eq_preset: true

**[Hand command in percentual] -> True , [Hand command in TICK] -> False**

hand_perc: true 

**[Current Reader Enable] -> True , [Current Reader Disable] -> False**

current: true

**Unit of measurement, ['DEG', 'RAD', 'TICK']**

unit: 'DEG' 

**Step time** 

step_time: 0.002

**ID cubes in chain**

IDcubes: [1, 2] 

**ID hands in chain**

IDhands: [3] 

# Communication
Depends on your configurations several topics will be open.
When IDcubes is set:

  - /qb_class/cube_ref
  - /qb_class/cube_measurement

  If "current" is true

  - /qb_class/cube_current
  
When IDhands is set:

  - /qb_class/hand_ref
  - /qb_class/hand_measurement
  
  If "current" is true
  - /qb_class/hand_current

**Message Type**

[/qb_class/cube_ref]

cubeRef.msg

[/qb_class/cube_measurement] 

When "eq_preset" is set the message type is cubeEq_preset.msg otherwise cubePos.msg

[/qb_class/cube_current] 

cubeCurrent.msg

[/qb_class/hand_ref]

handRef.msg

[/qb_class/hand_measurement] 

handPos.msg

[/qb_class/hand_current] 

handCurrent.msg
