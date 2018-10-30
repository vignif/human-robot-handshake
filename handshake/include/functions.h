/*
 * function.h
 *
 *  Created on: Jul 21, 2018
 *      Author: Francesco Vigni
 *
 * functions.h is a header file containing most common used function in the project
 * it embeds a class 'callbacks' that embeds the ros subscription to topics.
 *
 *
 */

#include <handRef.h>

#include <handPos.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <boost/tokenizer.hpp>
#include <vector>
#include <string>
#include <algorithm>    // copy
#include <iterator>
#include <ctime>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>

using namespace std;
using namespace boost;


static float value=0;
float Arr[6];
bool hand_detected=false;
float sensors_threshold=10;
std::vector<int> history;
class callbacks{
public:
	float closure;
	float error;
	float current;
	float smooth_current;
	float dummipalm_force;

	void cb_closure(const qb_interface::handRef::Ptr& msg){
		closure=msg->closure[0];
	}
	void cb_error(const qb_interface::handPos::Ptr& msg){
		error=msg->closure[1];
		}
	void cb_current(const qb_interface::handPos::Ptr& msg){
		current=msg->closure[2];
	}
	void cb_smooth_current(const std_msgs::Float32::Ptr& msg){
		smooth_current= (float)msg->data;
	}
	void cb_dummipalm(const std_msgs::Float32::ConstPtr& msg){
		dummipalm_force = msg->data;
	}
};

// read FSR sensor value from arduino and store them in a vector
void arrayCallback_sensors(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	std::setprecision(3);
	int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		Arr[i] = (int) (Arr[i] * 1000.0)/1000.0;
		i++;
	}
}


//minSensor =0
//maxSensor =5

float scale_closure(float value, float factor, int minSensor, int maxSensor) {
	int limit = 19000; //hand limit closure input
	for (int j = minSensor; j <= maxSensor; j++) {
		//printf("%f, ", Arr[j]);
		value += Arr[j];
	}
	value = value * factor;
	//printf("\n");
	if (value > limit) {
		value = limit;
	}
	return value;
}



void update_hand_status(){
	if( Arr[0]<sensors_threshold && Arr[1]<sensors_threshold){
		ROS_DEBUG_STREAM("hand_NOT_detected");
		hand_detected=false;
	}else{
		ROS_DEBUG_STREAM("hand_detected");
		//n.setParam("/stiffness",0.2); //publish parameter to ros
		hand_detected=true;
	}
}



//Funzione computeAVG NON UTILIZZATA
/*
std_msgs::Float32 computeAvg(std::vector<int> v){
	std_msgs::Float32 avg;
	avg.data=0;
	for (std::vector<int>::iterator it = v.begin() ; it != v.end(); ++it){
		avg.data += *it;
	}
	avg.data=avg.data/size;
	return avg;
}
 */



//funzione che ritorna la media mobile degli ultimi [window] valori
//function computes moving average of last [window] elements
//with current istantaneous value of chosen topic

std_msgs::Float32 historyAvg(int current,int window){
	std_msgs::Float32 avg;
	avg.data=0;
	int	sum=0;
	int rise_up_threshold=50;
	if(history.size()<window){
		history.push_back(current); //current from listener::current
	}else{
		history.erase (history.begin());  //erase first element
		history.push_back(current); //current from listener::current
	}
	cout <<"history size: "<< history.size() << endl;
	cout << "[";
	for (int i=0; i<=history.size()-1; i++){
		if(history[i]<rise_up_threshold){
			//std::fill(history.begin(), history.end(), 0);  //se anche solo un elemento della corrente e` inferiore alla soglia, saturo tutto il vettore a zero
			history[i]=0; //se il valore di corrente e` inferiore alla soglia lo saturo a zero
			sum=0;
		}else{
			sum+=history[i];
		}
		cout <<history[i] << ' ';
	}//esegui media del vettore
	cout <<"]" << endl;
	avg.data =sum/history.size();
	return avg;
}


float scale_controller1(float sumofFSR, int model[][2] , int minSensor, int maxSensor) {
	int limit = 19000; //hand limit closure input
	int output;
	for (int j = minSensor; j <= maxSensor; j++) {
		//printf("%f, ", Arr[j]);
		sumofFSR += Arr[j];
	}

	//load csv file and return value in which the sumofFSR index is found
	// model[position][sumofFSR];
	//model[][0] colonna posizione
	//model[][1] colonna sumofFSR
	clock_t begin = clock();
	//
	for (int i=0;i<19000;i++){
		if(abs(sumofFSR-model[i][1])<=2 ){
			cout << "FSRonline: " << sumofFSR << "  FSRfromModel: " << model[i][1] << endl;
			output=model[i][0];
			break;
		}else if(sumofFSR <1840){
			output=9000;
		}else if(sumofFSR >16040){
			output=limit;
		}
		//	if(model[i][1] < sumofFSR+3 && model[i][1] > sumofFSR){
		//		output=model[i][0];
		//	}
	}
	//	  cout <<"look for unknown value"<< model[19001][0] << endl;
	//	output =model[(int)sumofFSR][1];
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout <<"elapsed sec: "<< elapsed_secs << endl;
	//	output=val;
	if (output >= limit) {
		output = limit;
	}
	return output;
}


int compute_f_with_q0(int q0, int minS = 0, int maxS = 1){
float sumofFSR;
float p[4];
int q;
	for (int j = minS; j <= maxS; j++) {
		sumofFSR += Arr[j];
	}

	p[0] = 0.02432; p[1] =-2.862; p[2]= 157.2;

	q=p[0]*pow(sumofFSR,3)+p[1]*pow(sumofFSR,2)+p[2]*sumofFSR+p[3]+q0;

	return q;
}

bool check_contact(){
	float threshold =0.5;
	//Arr[2] is the sensor placed closed the thumb that triggers the handshake
	if (Arr[0]>threshold || Arr[1]>threshold || Arr[2] > threshold) {
		return true;
	}else{
		return false;
	}
}

bool fexists(const std::string& filename) {
  std::ifstream ifile(filename.c_str());
  return (bool)ifile;
}

static void show_usage(std::string name){
	std::cerr << "Usage:\n"
			<< "ctrl [handsize_code] [ID_participant]\n"
			<< "Usage: Input the HandSize code\n"
			<< "1(verybig hand)->5(verysmall hand1)\n"
			<< std::endl;
}


string save_file(const string& name) {
	/*save_file checks if a file exists in a specific folder
	it true create a new file with a incremental number in the name
	 */

	string dir = "/home/bot/ros_ws/ctrl/saves/";
	string filename = dir + name;
	int idx = 0;
	string filetosave;
	bool found = false;
	for (int idx = 0; idx < 10; idx++) {
		if (!fexists(filename + std::to_string(idx) + ".csv")) {
			filetosave = filename + std::to_string(idx) + ".csv";
			break;
		}
	}
	return filetosave;
}
