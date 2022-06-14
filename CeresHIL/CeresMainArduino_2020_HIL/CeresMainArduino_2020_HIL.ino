// ############################################################################################
// Script:        CeresMainArduino
// Version:       20.0
// Authors:       Adrien Legrand
// Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
// Proyecto:      Proyecto de Grado - 2018-II
// Goal:          Run an open-loop algorithm to control the CERES Agrobot
// Date:          29-10-2018
// ############################################################################################
//
//  SUMMARY:
//    I -     Libraries Including
//    II-     Parameters Definition
//    III-    Headers Definition
//    IV-     ROS Nodes Definition
//    V-      Global Variables Definition
//    VI-     Loop
//    VII-    Functions Declaration
// ############################################################################################

// ############################################################################################
// I-   LIBRARIES INCLUDING
// ############################################################################################
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ceres/CeresRC.h>
#include <ceres/CeresArduinoLogging.h>
#include "Timer.h"

#include <SPI.h>


// ############################################################################################
//  II-  PARAMETERS DEFINITION
// ############################################################################################
#define FREQ 40    // Pose input reference data frequence (Should also be modified in ROS)
#define cFREQ 10   // Controller frequency

#define CS_PIN 49

#define closeLoop false

#define autoTimeout 500 // Timeout for ROS Input
#define RCTimeout 1000 // Timeout for RC input
#define IMUTimeout 50 // Timeout for IMU Input

#define PI 3.1415926535897932384626433832795
#define R 0.29  // [m]
#define L 2.255 // [m]

#define Umin 1.46 // Minimum Drivers Voltage [V]
#define Umax 2.38 // Maximum Drivers Voltage [V]

#define Kl 67.6397556 // Left Driver Coefficient
#define Kr 67.2566484 // Right Driver Coefficient

#define ControlTime 20 //time for control interrupt
#define Pu 1.1
#define Iu 0.65
#define Du 0.0

#define Pw 0.8 
#define Iw 0.4 //0.1
#define Dw 0.0 //0.05

#define EM_BREAKER_PIN 25
#define PUMPS_PIN 23
#define EM_R_BR_PIN 35
#define EM_L_BR_PIN 37
#define R_BR_PIN 27
#define L_BR_PIN 29
#define R_REVERSE_PIN 31
#define L_REVERSE_PIN 33


#define minVel 0.0    // Maximum Robot Linear Velocity [m/s]
#define minAngVel -0.25 // Maximum Robot Angular Velocity [rad/s]

#define maxVel 1.0    // Maximum Robot Linear Velocity [m/s]
#define maxAngVel 0.25 // Maximum Robot Angular Velocity [rad/s]

#define maxManualVel 1.0    // Maximum Robot Linear Velocity in Manual Mode [m/s]
#define maxManualAngVel 0.25 // Maximum Robot Angular Velocity in Manual Mode [rad/s]

#define ImuNoiseLevel 0.02
#define WReverseThreshold 0.01

// ############################################################################################
// III-    HEADERS DEFINITION
// ############################################################################################
void receiveRefPose( const geometry_msgs::Twist& cmd_vel);
void receivePose(const geometry_msgs::Vector3& pose_msg);
void receiveRC(const ceres::CeresRC& rc_msg);
float PIDu(float Uin);
float PIDw(float Win);
void applySpeed(float u, float w);
void calculateClosedLoop();

// ############################################################################################
// IV-     ROS NODES DEFINITION
// ############################################################################################
ros::NodeHandle nh;
//ceres::CeresArduinoLogging log_msg;
geometry_msgs::Vector3 log_msg2;
//ros::Publisher log_pub("/ceres/arduLog", &log_msg);
ros::Publisher cs_pub("/ceres/controlSignal", &log_msg2);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("/ceres/cmd_vel", receiveRefPose);
ros::Subscriber<ceres::CeresRC> rc_sub("/ceres/RC", receiveRC);
ros::Subscriber<geometry_msgs::Vector3> pose_sub("/advanced_navigation_driver/twist2D", receivePose);
// ############################################################################################
// V-    GLOBAL VARIABLES DECLARATION
// ############################################################################################

bool notified = false;
bool IMUnotified = false;
int mode=2; // 0: RC/Manual, 1: Automatic, 2: Emergency Stop
int lastmode=-1;

bool leftReverse=false;
bool rightReverse=false;
long timeLeftReverse = 0;
bool leftReversing = false;
long timeRightReverse = 0;
bool rightReversing = false;

float Usens=0.0;
float Wsens=0.0;
float Uout=0;
float Wout=0;

unsigned long lastAutoReceivedTime=0;
unsigned long lastRCReceivedTime=0;
unsigned long lastIMUReceivedTime=0;

Timer tcontrol;

unsigned long lastControlTime=0;

//unsigned long lastPIDuTime=0;
float lastUerr=0.0;
//float Uerror=0.0;//for debug purpose
float IUerror=0;
bool USatFlag=false;

//unsigned long lastPIDwTime=0;
float lastWerr=0.0;
float Werror=0.0;
float IWerror=0;
bool WSatFlag=false;

int CH1=0;
int CH2=0;
int CH3=0;
int CH4=0;
int emergency=0;
int aux=0;

// SETUP
void setup() {  
      // Initialize DACs modules 
      SPI.begin();
      DAC_set(0, 1);
      DAC_set(0, 0);    
      
      nh.initNode();
      //nh.advertise(log_pub);
      nh.advertise(cs_pub);
      nh.subscribe(cmd_sub);
      nh.subscribe(rc_sub);
      nh.subscribe(pose_sub);  

      tcontrol.every(ControlTime, applycontrol, (void*)0);

      // Initialize Digital I/O
      pinMode(CS_PIN, OUTPUT);
      
      pinMode(PUMPS_PIN, OUTPUT);
      pinMode(EM_R_BR_PIN, OUTPUT);
      pinMode(EM_L_BR_PIN, OUTPUT);
      pinMode(R_BR_PIN, OUTPUT);
      pinMode(L_BR_PIN, OUTPUT);
      pinMode(R_REVERSE_PIN, OUTPUT);
      pinMode(L_REVERSE_PIN, OUTPUT);
      pinMode(EM_BREAKER_PIN, OUTPUT);

      digitalWrite(PUMPS_PIN, LOW);
      digitalWrite(EM_R_BR_PIN, HIGH);
      digitalWrite(EM_L_BR_PIN, HIGH);
      digitalWrite(R_BR_PIN, LOW);
      digitalWrite(L_BR_PIN, LOW);
      digitalWrite(R_REVERSE_PIN, LOW);
      digitalWrite(L_REVERSE_PIN, LOW);
      digitalWrite(EM_BREAKER_PIN, HIGH);
}

// ############################################################################################
// VI-   LOOP
// ############################################################################################
void loop() {
//if(digitalRead(ES_1_PIN) || digitalRead(ES_1_PIN)) {
//  mode = 0;
//}
  // Log Changed Mode
  if(lastmode!=mode) {
    switch(mode) {
      case(0):
        digitalWrite(EM_BREAKER_PIN, HIGH);
        nh.logwarn("[Main-Arduino] Manual/RC Mode Activated!");
        notified=false;
        IMUnotified=false;
        // LOW = FORWARD, HIGH=BACKWARD
        digitalWrite(R_REVERSE_PIN, HIGH);//defaul LOW=forward
        digitalWrite(L_REVERSE_PIN, HIGH);
        break;
      case(1):
        digitalWrite(EM_BREAKER_PIN, HIGH);
        nh.logwarn("[Main-Arduino] Automatic Mode Activated");
        notified=false;
        IMUnotified=false;
        break;
      case(2):
        IUerror=0.0;
        nh.logwarn("[Main-Arduino] Emergency Mode Activated!");
        notified=false;
        IMUnotified=false;
        break;
    }
    lastmode=mode;
  }

// ############################################################################################
  // Mode Detection  
  switch(mode) {
    case 0: // RC/Manual Mode
      if(millis()<lastRCReceivedTime+RCTimeout) { // Verify that a RC data has been received //
                                            //during the last 1000ms
          notified=false;
            
            if(closeLoop) {
              if(millis()<lastIMUReceivedTime+IMUTimeout) {
                tcontrol.update();
                IMUnotified = false;
              }
              else {
                applySpeed(0,0);
                if (!IMUnotified) {
                  nh.logerror("[Main-Arduino] IMU Receiving Timeout! Stopping Motors.");
                  IMUnotified = true;
                }
              }              
            }
            else{

            float u=CH1*maxManualVel/100;
            if(u<0) {
              u=0;
            }
            float w=-CH2*maxManualAngVel/100;
            
            applySpeed(u,w);
            //log_msg.Ul=u;
            //log_msg.Ur=(float)(millis());
            //log_msg.Vl=w;
            //log_msg.Vr=(USatFlag);
            //log_pub.publish(&log_msg);
            //nh.spinOnce();
            //lastControlTime=millis();
            //USatFlag=false;
            }
      }
      else {
          applySpeed(0,0);
          if (!notified) {
            nh.logerror("[Main-Arduino] RC Receiving Timeout! Stopping Motors.");
            notified=true;
          }
      }
      break;

    case 1: // Automatic Mode
      if(millis()<lastAutoReceivedTime+autoTimeout) { // Verify that a Open Loop data has been received  
                                            //during the last 500ms
          notified=false;
          if(closeLoop) {
              if(millis()<lastIMUReceivedTime+IMUTimeout) {
                tcontrol.update();
                IMUnotified = false;
              }
              else {
                applySpeed(0,0);
                if(!IMUnotified) {
                  nh.logerror("[Main-Arduino] IMU Receiving Timeout! Stopping Motors.");
                  IMUnotified = true;
                }
              }              
            }
            else{

            float u = Uout;
            float w = Wout;
            
            if(closeLoop) {
              u = PIDu(u);
              w = PIDw(w);
            }
            applySpeed(u,w);
            lastControlTime=millis();
            
         }
     }
      else {
          applySpeed(0,0);
          if (!notified) {
            nh.logerror("[Main-Arduino] ROS Receiving Timeout! Stopping Motors.");
            notified=true;
          }          
      }
      break;

    case 2:
        DAC_set(0, 1);
        DAC_set(0, 0);
        digitalWrite(EM_BREAKER_PIN, LOW);
  }
  nh.spinOnce();
}
// ############################################################################################
// VII-    FUNCTIONS DECLARATION
// ############################################################################################


void applycontrol(void *context)
{
    float u=0.0;
    float w=0.0;
    if(mode==0){
      u=CH1*maxManualVel/100;
      if(u<0) {
       u=0;
      }
      w=-CH2*maxManualAngVel/100;
    }
    else if(mode==1){
      u = Uout;
      w = Wout;
    }
    u = PIDu(u);
    w = PIDw(w);
    applySpeed(u,w);
    //log_msg.Ul=w;
    //log_msg.Ur=(float)(millis());
    //log_msg.Vl=u;
    //log_msg.Vr=(USatFlag);
    //log_pub.publish(&log_msg);
    //nh.spinOnce();
  //  lastControlTime=millis();
    USatFlag=false;
    WSatFlag=false;
}

// ############################################################################################
// 1. ReceiveRefPose: CallBack called each time a reference is received as a Twist message.
// ############################################################################################
void receiveRefPose( const geometry_msgs::Twist& cmd_vel){
  // Extraction of Positions/Orientation

  Uout=cmd_vel.linear.x;
  Wout=cmd_vel.angular.z;
  lastAutoReceivedTime=millis();
}


// ############################################################################################
// 2. ReceivePose: Callback called each time a Twist2D Message is received from the IMU.
// ############################################################################################
void receivePose(const geometry_msgs::Vector3& pose_msg) {
  Usens=pose_msg.x;
  Wsens=pose_msg.z;
  lastIMUReceivedTime=millis();
}


// ############################################################################################
// 3. ReceiveRC: CallBack called each time RC values are received as a CeresRC message.
// ############################################################################################
void receiveRC(const ceres::CeresRC& rrc_msg){
  //TODO: Verify data
  CH1=rrc_msg.CH1;
  CH2=rrc_msg.CH2;
  CH3=rrc_msg.CH3;
  CH4=rrc_msg.CH4;
  emergency=rrc_msg.emergency;
  aux=rrc_msg.AUX;
  if(emergency>=50) {
    mode=2;
  }
  else {
    if(aux<50) {
      mode=0;
    }
    else {
      mode=1;
    }
  }

  lastRCReceivedTime=millis();
}



// ############################################################################################
// 4. PIDu: Calculate the PID output value for U.
// ############################################################################################
float PIDu(float Uin) {
  
  float Uerror = Uin - Usens;
  unsigned long newTime=millis();
  float Upu = Pu * Uerror;
  float Uiu = (Iu * Uerror * ControlTime / 1000.0) + IUerror;
  float Udu = Du * (Uerror - lastUerr) * 1000.0 /ControlTime;
  float Uout = Upu + Uiu + Udu;
  if(Uout > 0.8) {
    Uout = 0.8;
    USatFlag=true;
  }
  if(Uout < -0.8) {
    Uout = -0.8;
    USatFlag=true;
  }
  if(USatFlag){
    lastUerr=lastUerr;
    IUerror=IUerror;
  }
  else{
    lastUerr = Uerror;
    IUerror=Uiu;
  }
  return Uout;
}


// ############################################################################################
// 5. PIDw: Calculate the PID output value for W.
// ############################################################################################
float PIDw(float Win) {
  
  float error = Win - Wsens;
  unsigned long newTime=millis();
  float Wpu = Pw * error;
  float Wiu = (Iw * error * ControlTime / 1000.0) + IWerror;
  float Wdu = Dw * (error - lastWerr) * 1000.0 /ControlTime;
  float Wout = Wpu + Wiu + Wdu;
  if(Wout > 0.3) {
    Uout = 0.3;
    WSatFlag=true;
  }
  if(Wout < -0.3) {
    Wout = -0.3;
    WSatFlag=true;
  }
  if(WSatFlag){
    lastWerr=lastWerr;
    IWerror=IWerror;
  }
  else{
    lastWerr = error;
    IWerror=Wiu;
  }
  return Wout;
}

// ############################################################################################
// 6. applySpeed: 1st-order transfer function that convert the speeds to drivers voltages 
//                and apply them
// ############################################################################################
void applySpeed(float u, float w) {
  // Verify that the speed is in the range
  if(u > maxVel) {
    u = maxVel;
  }
  if(w > maxAngVel) {
    w = maxAngVel;
  }

  if(u < minVel) {
    u = minVel;
  }
  if(w < minAngVel) {
    w = minAngVel;
  }
  
  // Convert from robot X-linear and Z-angular speed to robot's wheels speeds: Wl, Wr
  float Wl=(2*u-w*L)/(2*R);
  float Wr=(2*u+w*L)/(2*R);

  // Reverse Managment

/* // <------! START REVERSE 2 SECONDS METHOD !------>
  if((leftReversing && millis()>timeLeftReverse+2000) || (Wl<0 && leftReverse) || (Wl>0 && !leftReverse)) {
    leftReversing = false;
    digitalWrite(L_BR_PIN, LOW);
  }
 
  else if((Wl<0 && !leftReverse) || (Wl>0 && leftReverse)) {
    // If Wheel Theorical Speed is oposed to Wheel Direction (Reverse)
    Wl=0.0;
    timeLeftReverse=millis();
    leftReversing=true;
    digitalWrite(L_BR_PIN, HIGH);
  }

  if(Wl < 0) {
    Wl=-Wl;
    if(!leftReverse) {
      leftReverse=true;
      digitalWrite(L_REVERSE_PIN, HIGH);
    }
  }
  else if (Wl > 0) {
    if(leftReverse) {
      leftReverse=false;
      digitalWrite(L_REVERSE_PIN, LOW);
    }
  }



  if((rightReversing && millis()>timeRightReverse+2000) || (Wr<0 && rightReverse) || (Wr>0 && !rightReverse)) {
    rightReversing = false;
    digitalWrite(R_BR_PIN, LOW);
  }
 
  else if((Wr<0 && !rightReverse) || (Wr>0 && rightReverse)) {
    // If Wheel Theorical Speed is oposed to Wheel Direction (Reverse)
    Wr=0.0;
    timeRightReverse=millis();
    rightReversing=true;
    digitalWrite(R_BR_PIN, HIGH);
  }

  if(Wr < 0) {
    Wr=-Wr;
    if(!rightReverse) {
      rightReverse=true;
      digitalWrite(R_REVERSE_PIN, HIGH);
    }
  }
  else if (Wr > 0) {
    if(rightReverse) {
      rightReverse=false;
      digitalWrite(R_REVERSE_PIN, LOW);
    }
  }
   // <------! END REVERSE 2 SECONDS METHOD !------>
*/
   
  // Convert from robot's wheels speeds Wl, Wr to Driver Input Voltages Ul, Ur
  float Ul=0.0;
  float Ur=0.0;

  Ul=((30*Wl)/3.14)/Kl+Umin;
  Ur=((30*Wr)/3.14)/Kr+Umin;

  // Verify that calculated volatges are in the Driver input range [Umin;Umax].
  if(Ul<=Umin) {
    Ul=Umin-0.05;
  }
  if(Ur<=Umin) {
    Ur=Umin-0.05;
  }

  if(Ul>Umax) {
    Ul=Umax+0.05;
  }

  if(Ur>Umax) {
    Ur=Umax+0.05;
  }

  // Publish Robot wheels speed Wl, Wr and drivers input volages Ur, Ul into a ROS topic.    GSH moved to switch mode for debugging
//  log_msg.Ul=Ul;
//  log_msg.Ur=Ur;
//  log_msg.Vl=u;
//  log_msg.Vr=w;
//  log_pub.publish(&log_msg);
//  nh.spinOnce();
  
  log_msg2.z=w;
  log_msg2.y=0.0;
  log_msg2.x=u;
  cs_pub.publish(&log_msg2);
  nh.spinOnce();
  
  // Convert from Driver input voltages Ul, Ur to DAC Increments
  float Discl = 0.0;
  float Discr = 0.0;
  if(Ul>=1.5) {
    Discl = 1994.9891441755*Ul-2880.6841986894;
  }

  if(Ur>=1.5) {
    Discr = 2007.2889718494*Ur-2877.7740013957;
  }

  // Apply command to DACs.
  DAC_set(Discl, 0);
  DAC_set(Discr, 1);
}



void DAC_set(unsigned int input, int DAC_sel)
{
  //DAC_sel choose which DAC channel you want to write to A or B
  //Gain_sel choose your gain: H=2xVref and L=1xVref
  byte MSB,LSB;//most sig, least sig bytes and config info

  //only run the rest of the code if binary is in range.
  if (input>=0 && input<=4095)
  {
  //convert decimal input to binary stored in two bytes
  MSB = (input >> 8) & 0xFF;  //most sig byte
  LSB = input & 0xFF;         //least sig byte
  
  //apply config bits to the front of MSB
  if (DAC_sel==0)
    MSB &= 0x7F; //writing a 0 to bit 7.
  else if (DAC_sel==1 || DAC_sel==1)
    MSB |= 0x80; //writing a 1 to bit 7.

    MSB &= 0xDF;

  //get out of shutdown mode to active state
  MSB |= 0x10;

  //now write to DAC
  // take the CS pin low to select the chip:
  digitalWrite(CS_PIN,LOW);
  //delay(1);
  //  send in the address and value via SPI:
  SPI.transfer(MSB);
  SPI.transfer(LSB);
  //delay(1);
  // take the CS pin high to de-select the chip:
  digitalWrite(CS_PIN,HIGH);
  }
}
