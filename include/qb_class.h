// Define
#define ERR_TIMEOUT		5
#define EQ_PRESET		true
#define PERC			true

// ROS Headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

// Custom ROS messages
#include <qb_interface/cubeRef.h>
#include <qb_interface/handRef.h>

#include <qb_interface/cubeEq_Preset.h>
#include <qb_interface/cubePos.h>
#include <qb_interface/handPos.h>

#include <qb_interface/cubeCurrent.h>
#include <qb_interface/handCurrent.h>

// General Headers
#include <vector>
#include <map>
#include <iostream>

// Custom Headers
#include "qbCube.h"
#include "qbHand.h"

using namespace std;


class qb_class{

	public:
		// Costructor
		qb_class();

		// Destructor
		~qb_class();

		// SpinOnce function
		void spinOnce();

		// Spin function
		void spin();

	protected:
		// Open communication
		bool open(const char*, const int);

		// Close communication
		bool close();

		// Communication structure
		comm_settings* qb_comm_;

		// Step Time, 1 / step_time = communication frequency
		double step_time_;

	private:

		// Functions

		// Activation function
		bool activate();

		// Deactivation function
		bool deactivate();

		// Read positions 
		bool readMeas();

		// Read positions and currents
		bool readMeasCurrent();

		// Move cubes and hands
		void move();

		// Get current of cubes and hands
		bool readCurrent();

		// Callback functions for referiments 
		void cubeRefCallback(const qb_interface::cubeRef::ConstPtr&);
		void handRefCallback(const qb_interface::handRef::ConstPtr&);

		// Post on topic functions
		void sendHandMeas(vector<float>);
		void sendCubeMeas(vector<float>, vector<float>);
		void sendCubeMeas(vector<float>, vector<float>, vector<float>);

		void sendCurrent(vector<int>);
		void sendCurrent(vector<int>, vector<int>);
		
		// Variables

		vector<qbCube*> cube_chain_;
		vector<qbHand*> hand_chain_;

		vector<float> p_1_, p_2_;
		vector<float> pos_;

		// Configurations

		// Vector of ID with request current of cube and hand

		vector<int> current_cube_;
		vector<int> current_hand_;

		// [Eq./Preset] <- true or [Pos_1/Pos_2] <- false flag

		bool flagCMD_type_;
		
		// [TICK] <- true or [Perc.] <- false flag

		bool flag_HCMD_type_;

		// Activate current option

		bool flag_curr_type_;

		// Measurements Unit [DEG-RAD-TICK]

		angular_unit meas_unit_;



		// Ros node handle

		ros::NodeHandle* node_;

		// Subscriber Variables
 		ros::Subscriber cube_sub;
 		ros::Subscriber hand_sub;

 		// Publisher variables
		ros::Publisher cubeRef_pub;
		ros::Publisher handRef_pub;

		ros::Publisher cube_pub;
		ros::Publisher hand_pub;

		ros::Publisher cube_curr_pub;
		ros::Publisher hand_curr_pub;

};