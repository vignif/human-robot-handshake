/******************************************************************************
  FSR_qb_interface.ino
  starting from:
  Example sketch for SparkFun's force sensitive resistors
  (https://www.sparkfun.com/products/9375)
  Jim Lindblom @ SparkFun Electronics   April 28, 2016

function ComputeForce with specific:
  Create a voltage divider circuit combining an FSR with a 3.3k resistor.
  - The resistor should connect from A0 to GND.
  - The FSR should connect from A0 to 3.3V
  As the resistance of the FSR decreases (meaning an increase in pressure), the
  voltage at A0 should increase.

ROS integration:
  publisher 'chatter' is publishing a topic in ROS environment with a vector of float32 of size = NumberOfSensors,
  named 'sensors_FSR' where each entry is the force sensor value read by the pins.

The parameter NumberOfSensors is currently set to 4 but if needed can be changed in order to publish a smaller/larger
vector. 


Francesco Vigni @ SIRSLab
july 8, 2018
 
******************************************************************************/

#include <ros.h>
/*
#include <qb_interface/handRef.h> //includi custom message in modo
//che arduino invii direttamente a ros il valore closure.
//path per includere libreria ~/Arduino/libraries/Rosserial_Arduino_Library/src/
*/
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;
const int queue_size=0; //infinit queue can overload arduino memory if the not dequeued properly
//create ros publisher named 'sensors_FSR'
std_msgs::Float32MultiArray sensors;
ros::Publisher chatter("sensors_FSR_2_delay", &sensors, queue_size);


//publisher for closure
/*
qb_interface::handRef ClosureValue;
ros::Publisher send_closure("closure", &ClosureValue, queue_size);
float value;
*/
// Modify these values if needed
//start
const int NumberOfSensors =3; 
const float threshold=3; // minimum pressure filtered
//end

int fsrADC[NumberOfSensors];       //initialize Int array
int FSR_PIN[]={A0,A1,A2,A3,A4,A5}; //create array of PINS


float ComputeForce(int fsrADC){
  // This function is called for each sensor, and is returning a force based on FSR datasheet. 
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  // Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
  // their value's below:
    const float VCC = 4.98; // Measured voltage of Ardunio 5V line
    const float R_DIV = 3230.0; // Measured resistance of 3.3k resistor
 
    float force;
    float fsrV = fsrADC * VCC / 1023.0;
 // Use voltage and static resistor value to
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    // Guesstimate force based on slopes in figure 3 of FSR datasheet:
    
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    force =  fsrG / 0.00000048462;
    float x=force;
    float p1=0.000000002863;
    float p2=-0.00001851;
    float p3=0.04863;
    float model =p1*x*x*x+p2*x*x+p3*x;
    return model;
    }

void setup()
{
  nh.getHardware()->setBaud(57600);
  pinMode(FSR_PIN[0], INPUT);
  
  nh.initNode();
  nh.advertise(chatter);
//  nh.advertise(send_closure);  
  sensors.data_length=NumberOfSensors;
  //allocate memory for the array
  sensors.data = (float *)malloc(sizeof(float)*NumberOfSensors);
}

void loop()
{
  
 //for loop over sensors, and compute force from analog reading with function ComputeForce
for(int i =0; i<NumberOfSensors; i++){
  fsrADC[i]=analogRead(FSR_PIN[i]); // fill array of fsrADC from analogReads
  if(fsrADC[i]>threshold){ //check if threshold is satisfied
  sensors.data[i]=ComputeForce(fsrADC[i]); //compute force from analog read with function ComputeForce
  
  }else{
  sensors.data[i]=0; // if the threshold is not satistied set the value to zero
  }
  }
//delay
//Start publish routine
chatter.publish( &sensors ); //i'm publishing the sensors pointer to array
nh.spinOnce();
delayMicroseconds(1000);
//END publish routine
}
