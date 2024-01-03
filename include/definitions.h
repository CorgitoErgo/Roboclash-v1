#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

const double FORWARD_KP = 1.2;
const double FORWARD_KI = 0.002;
const double FORWARD_KD = 0.0002;

const double TURN_KP = 4.2;
const double TURN_KI = 0.003;
const double TURN_KD = 0.001;

#define LEFT_MOTOR_FRONT_PORT 10
#define LEFT_MOTOR_REAR_PORT 1

#define RIGHT_MOTOR_FRONT_PORT 20
#define RIGHT_MOTOR_REAR_PORT 11

#define TOP_ROLLER_MOTOR_PORT 9
#define BOTTOM_ROLLER_MOTOR_PORT 19

#define SORTING_MOTOR_PORT 18
#define OPTICAL_PORT 8
#define OPTICAL_PORT_UPDATE 20
#define IMU_SENSOR_PORT 7

#define Math_PI 3.1415926

const int TICKS_PER_REVOLUTION = 360;
//const double WHEEL_DIAMETER_CM = 7.0; //when using green tracking wheels
const double WHEEL_DIAMETER_CM = 10.16;
const double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * Math_PI;
const double CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REVOLUTION;

#endif