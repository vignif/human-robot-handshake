## Human-Robot Handshake 
Using Pisa/IIT SoftHand node [qb_interface_node], and attaching  fsr sensors to the side of the robot hand as described in [ref] it is possible to set up a closed loop control of the robot force, from the human force.

[qb_interface_node] package for Pisa/IIT SoftHand position control

[rosserial] package for enabling Arduino Uno to transmit data over ROS, it is used for reading values from fsr

[handshake] package includes 5 different controllers for human-robot handshake. 

[arduino] contains the source file that must be flashed on the Arduino board in order to manage the fsr sensors


# controllers

the package [handshake] contains:

node [find_q0.py] sends to the robot hand different closure position, the user should communicate when the robot hand is softly touching the hand. we define that position as [q0]. 'starting position of the handshake', a code of the handsize will be provided and must be used in the controllers.

The implemented controllers named [ctrl_i] i=1:5 must run as i.e.

[rosrun handshake ctrl_<idcontroller> <handsizecode> <userid>]

================================================================

the file in [handshake/include/functions.h] contains the saving path for the experiments,
if you want to save files acquiring data from the experiment modify the path.

[rosrun handshake delaynode.py] is delaying the FSRs signal of 120ms. 

================================================================

# compile
before cloning this repo, create a catkin workspace, f.i. [ros_ws]
type in a terminal:
cd ros_ws

now you are in the folder /ros_ws with the cmd line. execute:
git clone https://github.com/vignif/qb_interface_node.git

rename the folder 'qb_interface_node_src' in 'src'
type in the terminal:
catkin_make



# run

rosparam load $(rospack find qb_interface)/conf/config.yaml


