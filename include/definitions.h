#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#define RIGHT_MOTOR_FRONT_PORT 20
#define TOP_ROLLER_MOTOR_PORT 9

//#define LEFT_MOTOR_FRONT_PORT 1 //TESTING SMALL ROBOT WITH 3 TRACKING WHEELS
//#define RIGHT_MOTOR_FRONT_PORT 9 //^^

#define BOTTOM_ROLLER_MOTOR_PORT 19
#define LEFT_MOTOR_FRONT_PORT 10
#define RIGHT_MOTOR_REAR_PORT 11
#define LEFT_MOTOR_REAR_PORT 1
//#define INERTIA_SENSOR_PORT 6
#define SORTING_MOTOR_PORT 18
#define OPTICAL_PORT 8
#define OPTICAL_PORT_UPDATE 20
#define VISION_SENSOR_PORT 5
//#define base_kp 4
//#define base_kd 0.005
//#define base_ki 0.015
//#define base_error 25
#define Math_PI 3.141592

/* PID */
// PID constants for forward/reverse motion
double FORWARD_KP = 1.9;  // Proportional constant
double FORWARD_KI = 0.001;  // Integral constant
double FORWARD_KD = 0.00012;  // Derivative constant

// PID constants for turning
double TURN_KP = 4.1;  // Proportional constant
double TURN_KI = 0.003;  // Integral constant
double TURN_KD = 0.001;  // Derivative constant

const int TICKS_PER_REVOLUTION = 360;
const double WHEEL_DIAMETER_CM = 7.0; //using green tracking wheels
//const double WHEEL_DIAMETER_CM = 10.16; //using motor's built in encoder (grey wheels)
const double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * Math_PI;
const double CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REVOLUTION;
/* END PID */

//const int LEFT_TRACKING_ERROR = 5;
//const int RIGHT_TRACKING_ERROR = 10;
//const int TRACKING_WHEEL_DIAMETER = 7; //cm, 2.75 inches
//const int TRACKING_TICKS_PER_CM = 360/7;
double turn_pid_output = 0;
double forward_pid_outputL = 0;
double forward_pid_outputR = 0;
int page = 0;
double auton_fb_target = 0;
double auton_turn_target = 0;

#endif