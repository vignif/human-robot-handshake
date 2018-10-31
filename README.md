# Human-Robot Handshake 
Using Pisa/IIT SoftHand node [qb_interface_node], and attaching  fsr sensors to the side of the robot hand as described in [ref] it is possible to set up a closed loop control of the robot force.

[qb_interface_node] package for Pisa/IIT SoftHand position control

[rosserial] package for enabling Arduino Uno to transmit data over ROS, it is used for reading values from fsr

[handshake] package includes 5 different controllers for human-robot handshake. 

[arduino] contains the source file that must be flashed on the Arduino board in order to manage the fsr sensors

### DOWNLOAD & compile
before cloning this repo, create a catkin workspace, f.i. [ros_ws]
type in a terminal:
cd ros_ws

now you are in the folder /ros_ws with the cmd line. execute:
git clone https://github.com/vignif/qb_interface_node.git

rename the folder [qb_interface_node] in [src]
type in the terminal:
catkin_make


### controllers

the package [handshake] contains:

node [find_q0.py] sends to the robot hand different closure position, the user should communicate when the robot hand is softly touching his/her hand; key 'x' on the keyboard should be pressed; the program stops define the last position as [q0]. 'starting position of the handshake' is a function of the handsize and must be used in the controllers aka <handsizecode>.

The implemented controllers named [ctrl_i] i=1:5 must run as i.e.

[rosrun handshake ctrl_<idcontroller> <handsizecode> <userid>]

================================================================

the file in [handshake/include/functions.h] contains the saving path for the experiments,
if you want to save files acquiring data from the experiment modify the local path inside the file.

[rosrun handshake delaynode.py] is delaying the FSRs signal of 120ms. 

================================================================


### run
terminal 1:

roscore


terminal 2: (robot hand)

rosparam load $(rospack find qb_interface)/conf/config.yaml

rosrun qb_interface qb_force_control

this is the node for the robot hand, for more details see README.md inside folder [qb_interface_node]


terminal 3: (sensors)

rosrun rosserial_python serial_node.py /dev/ttyACM0


terminal 4: (delay the sensors)

rosrun handshake delaynode.py


terminal 5: (controller)

[rosrun handshake ctrl_<idcontroller> <handsizecode> <userid>]

================================================================

load the source in all used terminals, type:

nano .bashrc

go to the end of the file and add the following line:

source ~/ros_ws/devel/setup.bash


### IDE eclipse

in order to setup eclipse for developing a ros project you have to:
open a terminal type:

cd /ros_ws

catkin_make --force-cmake -G "Eclipse CDT4 - Unix Makefiles"


### ROS plots
plot the force resistors values
open a terminal and type: 		rqt_plot /sensors_FSR_2_delay/data[0] /sensors_FSR_2_delayata[1]

