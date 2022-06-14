// ############################################################################################
// Script:        Arduino_LazoAbierto
// Version:       6.0
// Authors:       Adrien Baptiste Legrand, Claudia Ospina, Daniel Montenegro, Santiago Bonilla
// Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
// Proyecto:      PAI Agrobot CERES 2018-I
// Goal:          Run an open-loop algorithm to control the CERES Agrobot
// Date:          10-05-2018
// ############################################################################################
// INCLUDES // DEFINES
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif


//#include <Scheduler.h>
// ############################################################################################
// ROS Messages Includes
#include <ros.h>

#include <ceres/CeresRC.h>
#include <ceres/CeresArduinoLogging.h>

// ############################################################################################
// RCRead Defines

#define channumber 4      // Channels Number
#define zeroThRange 150   // Zero Range
#define mednumber 10      // Number of iterations for a medition
// ############################################################################################
// Headers definitions:
void readServo();
void process();
void calibration();
// ############################################################################################
// ROS Global Variables Declaration
ros::NodeHandle nh;

ceres::CeresRC rc_msg;
ros::Publisher rc_pub("/ceres/RC", &rc_msg);
// ############################################################################################
// RCRead Global Variables Declaration
long minTh[channumber];   // min Value for RC Channels
long maxTh[channumber];   // max Value for RC Channels
long zeroTh[channumber];  // zero Value for RC Channels
long channel[channumber]; // Medition Values
long value[channumber];   // Processed Values
//int pin[] = {2, 3, 4, 5, 6, 7, 8, 9};      // RC Pins
int pin[] = {3, 9, 7, 6};
// ############################################################################################
// SETUP
void setup() {  
      nh.initNode();
      nh.advertise(rc_pub);
      calibration(); // Calibrate RC.
}
// ############################################################################################
// LOOP
void loop() {
  readServo();
  process();
  rc_msg.CH1=(int)value[0];
  rc_msg.CH2=(int)value[1];
  rc_msg.CH3=0;
  rc_msg.CH4=0;
  rc_msg.emergency=value[2];
  rc_msg.AUX=value[3];
  rc_pub.publish(&rc_msg);
  nh.spinOnce();
}
