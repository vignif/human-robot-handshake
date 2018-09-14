#include <qb_force_control.h>
#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <functions.h>





class Listener{
public:
	int current;
	void currCallback(const qb_interface::handPos::Ptr& msg);
};


void Listener::currCallback(const qb_interface::handPos::Ptr& msg){
	current = msg->closure[2];
	//ROS_DEBUG("current %d",current);
	ROS_DEBUG_STREAM("closure: " << msg->closure[0] << "\tcurrent:" << current);
}


int main(int argc, char **argv)
{
	std_msgs::Float32 media;
	std_msgs::Float32MultiArray array;

	cout << "filter node started" << endl;
	//state.closure.clear();
	ros::init(argc, argv, "FSR_current_control");
	ros::NodeHandle n;
	//read 6 arduino sensors
	ros::Subscriber sub = n.subscribe("sensors_FSR", 100, arrayCallback_sensors);

	Listener listen;

	//read current from qb_interface
	ros::Subscriber sub_current = n.subscribe("/qb_class/hand_measurement", 100, &Listener::currCallback, &listen);

	//publish closure value
	ros::Publisher pub = n.advertise<qb_interface::handRef>("/qb_class/hand_ref", 100);

	//publish average
	ros::Publisher pub1 = n.advertise<std_msgs::Float32>("smooth",100);

	qb_interface::handPos state;
	//ros::Rate loop_rate(10); //10hz

	float value_past=0;
	int history_window=10;
	while (ros::ok())
	{
		state.closure.clear();
		media.data = 0;

		ros::spinOnce();
		value = scale_closure(value, 3.4, 0, 5);

		//compute average of last [history_window] element in the vector from current
		media = historyAvg(listen.current, history_window);

		pub1.publish(media);
		cout <<"current \t"<< listen.current << endl;
		cout <<"average \t"<< media.data<< endl;
		//	cout << media << endl;

		state.closure.push_back((int)value);
		//n.setParam("/stiffness",0.9); //publish parameter to ros
		//pub.publish(state);
		usleep(50000);  //dynamic usleeps takes microseconds in input

		//end arduino values

	}
}
