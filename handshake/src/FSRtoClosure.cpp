#include <qb_force_control.h>
#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <functions.h>


int main(int argc, char **argv)
{

	ros::init(argc, argv, "FSRtoClosure");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("sensors_FSR", 100, arrayCallback_sensors);
	ros::Publisher pub = n.advertise<qb_interface::handRef>("/qb_class/hand_ref", 100);
	qb_interface::handPos state;

	ROS_INFO("FSRtoClosure node started");

	//scaling_value =600 if fsr sensors are calibrated with N
	//scaling_value = 4 if fsr sensors are not calibrated with N
	int scaling_value=600;
	ros::param::set("/stiffness",1.0);

	while (ros::ok())
	{
		float value=0;
		ros::spinOnce();
		state.closure.clear();
		value = scale_closure(value, scaling_value, 0 , 4);
		//round the closure value to the closest integer
		//since the topic reads integers
		state.closure.push_back((int)value);

		pub.publish(state);
		usleep(10000);  //dynamic usleeps takes microseconds in input
	}
}
