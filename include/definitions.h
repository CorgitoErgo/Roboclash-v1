#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

const double FORWARD_KP = 0.61;
const double FORWARD_KI = 0;
const double FORWARD_KD = 0.5;

const double TURN_KP = 0.4;
const double TURN_KI = 0.0015;
const double TURN_KD = 0.01;

#define LEFT_MOTOR_FRONT_PORT 11
#define LEFT_MOTOR_REAR_PORT 20

#define RIGHT_MOTOR_FRONT_PORT 1
#define RIGHT_MOTOR_REAR_PORT 10

#define TOP_ROLLER_MOTOR_PORT 12
#define BOTTOM_ROLLER_MOTOR_PORT 2

#define SORTING_MOTOR_PORT 13
#define OPTICAL_PORT 15
#define OPTICAL_PORT_UPDATE 20
#define IMU_SENSOR_PORT 17

#define EVAC_MOTOR_PORT 3

#define Math_PI 3

const int TICKS_PER_REVOLUTION = 360;
//const double WHEEL_DIAMETER_CM = 7.0; //when using green tracking wheels
const double WHEEL_DIAMETER_CM = 10;
const double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * Math_PI;
const double CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / TICKS_PER_REVOLUTION;

#endif