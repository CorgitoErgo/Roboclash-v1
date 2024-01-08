#include "main.h"

bool sorting_enable = true;
int Top_Roller_State = 3;
int Btm_Roller_State = 3;
double imu_error = 0;

/*Check definitions header file for port assignments*/
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor LEFT_MOTOR_FRONT(LEFT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_FRONT(RIGHT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_REAR(RIGHT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LEFT_MOTOR_REAR(LEFT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor SORTING_MOTOR(SORTING_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BTM_ROLLER_MOTOR(BOTTOM_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor TOP_ROLLER_MOTOR(TOP_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor EVAC_MOTOR(EVAC_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Optical OPTICAL_SENSOR(OPTICAL_PORT, OPTICAL_PORT_UPDATE);
pros::Imu IMU_SENSOR(IMU_SENSOR_PORT);
pros::c::optical_rgb_s_t rgb_value;

void forwardPID(double target_cm) {
	double currentPositionLeft = 0, currentPositionRight = 0;
	double errorL = 0, integralR = 0, derivativeR = 0, lastErrorR = 0, powerR = 0;
	double errorR = 0, integralL = 0, derivativeL = 0, lastErrorL = 0, powerL = 0;
	//double error = 0, integral = 0, derivative = 0, lastError = 0, power = 0, currentPosition = 0;
	double target = fabs(target_cm) / CM_PER_TICK;
	double correctingPower = 0;
	bool forward = true;
	bool leftMotorRun = true;
	bool rightMotorRun = true;

	LEFT_MOTOR_FRONT.tare_position();
	RIGHT_MOTOR_FRONT.tare_position();
	LEFT_MOTOR_REAR.tare_position();
	RIGHT_MOTOR_REAR.tare_position();
	IMU_SENSOR.tare();

	if(target_cm<0) forward = false;

    while (1) {
        //currentPositionLeft = (fabs(LEFT_MOTOR_FRONT.get_position()) + fabs(LEFT_MOTOR_REAR.get_position()))/2;
		//currentPositionRight = (fabs(RIGHT_MOTOR_FRONT.get_position()) + fabs(RIGHT_MOTOR_REAR.get_position()))/2;
		//currentPosition = (currentPositionLeft + currentPositionRight)/2;
		currentPositionLeft = LEFT_MOTOR_FRONT.get_position();
		currentPositionRight = RIGHT_MOTOR_FRONT.get_position();
		//currentPosition = (currentPositionLeft + currentPositionRight)/2;
		correctingPower = IMU_SENSOR.get_heading()/2;

		errorL = target - currentPositionLeft;
		errorR = target - currentPositionRight;

		integralL += errorL;
		integralR += errorR;

		derivativeL = errorL - lastErrorL;
		derivativeR = errorR - lastErrorR;

		powerL = ((FORWARD_KP * errorL) + (FORWARD_KI * integralL) + (FORWARD_KD * derivativeL));
		powerR = ((FORWARD_KPR * errorR) + (FORWARD_KIR * integralR) + (FORWARD_KDR * derivativeR));


		if(forward==true){
			if(leftMotorRun == true){
				LEFT_MOTOR_FRONT.move_velocity(powerL+correctingPower);
				LEFT_MOTOR_REAR.move_velocity(powerL+correctingPower);
			}
			if(rightMotorRun == true){
				RIGHT_MOTOR_FRONT.move_velocity(powerR-correctingPower);
				RIGHT_MOTOR_REAR.move_velocity(powerR-correctingPower);
			}
		}
		else{
			if(leftMotorRun == true){
				LEFT_MOTOR_FRONT.move_velocity(-(powerL+correctingPower));
				LEFT_MOTOR_REAR.move_velocity(-(powerL+correctingPower));
			}
			if(rightMotorRun == true){
				RIGHT_MOTOR_FRONT.move_velocity(-(powerR-correctingPower));
				RIGHT_MOTOR_REAR.move_velocity(-(powerR-correctingPower));
			}
		}
		lastErrorL = errorL;
		lastErrorR = errorR;

		/*if(fabs(error) <= 3 || (currentPositionLeft >= target && currentPositionRight >= target)){
			LEFT_MOTOR_FRONT.move_velocity(-10);
			LEFT_MOTOR_REAR.move_velocity(-10);
			RIGHT_MOTOR_FRONT.move_velocity(-10);
			RIGHT_MOTOR_REAR.move_velocity(-10);
			pros::delay(5);
			LEFT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_REAR.brake();
			break;
		}*/
		if(fabs(errorL) <= 3){
			leftMotorRun = false;
			//LEFT_MOTOR_FRONT.move_velocity(0);
			//LEFT_MOTOR_REAR.move_velocity(0);
			LEFT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
		}
		if(fabs(errorR) <= 3){
			rightMotorRun = false;
			//RIGHT_MOTOR_FRONT.move_velocity(0);
			//RIGHT_MOTOR_REAR.move_velocity(0);
			RIGHT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_REAR.brake();
		}
		if(leftMotorRun == false || rightMotorRun == false){
			break;
		}
		//master.print(2,0,"%lf",currentPositionL);
        pros::delay(2);
    }
	pros::delay(20);
}

void turnPID(double target_degrees, bool turn_left = true) {
    double error = 0, integral = 0, derivative = 0, lastError = 0, currentRotation = 0;
	double currentHeading = 0;
	double power = 0;
	double offset_power = 0;
	double target = target_degrees;

	//LEFT_MOTOR_FRONT.tare_position();
	//RIGHT_MOTOR_FRONT.tare_position();
	//LEFT_MOTOR_REAR.tare_position();
	//RIGHT_MOTOR_REAR.tare_position();
	IMU_SENSOR.tare();
	//imu_error = IMU_SENSOR.get_rotation();

    while (1) {
		currentHeading = fabs(IMU_SENSOR.get_rotation());
        error = target - currentHeading;
        integral += error;
        derivative = error - lastError;

        power = (TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative);
		
		if(turn_left==true){
			LEFT_MOTOR_FRONT.move_velocity(-power-offset_power);
			RIGHT_MOTOR_FRONT.move_velocity(power);
			LEFT_MOTOR_REAR.move_velocity(-power-offset_power);
			RIGHT_MOTOR_REAR.move_velocity(power);
		}
		else{
			LEFT_MOTOR_FRONT.move_velocity(power+offset_power);
			RIGHT_MOTOR_FRONT.move_velocity(-power);
			LEFT_MOTOR_REAR.move_velocity(power+offset_power);
			RIGHT_MOTOR_REAR.move_velocity(-power);
		}
        lastError = error;
		currentHeading = fabs(IMU_SENSOR.get_rotation());
		error = target - currentHeading;

        if ((fabs(error) <= 0.05) || currentHeading >= target) {
			/*if(turn_left){
				LEFT_MOTOR_FRONT.move_velocity(-2);
				LEFT_MOTOR_REAR.move_velocity(-2);
				RIGHT_MOTOR_FRONT.move_velocity(2);
				RIGHT_MOTOR_REAR.move_velocity(2);
			}
			else{
				LEFT_MOTOR_FRONT.move_velocity(2);
				LEFT_MOTOR_REAR.move_velocity(2);
				RIGHT_MOTOR_FRONT.move_velocity(-2);
				RIGHT_MOTOR_REAR.move_velocity(-2);
			}
			pros::delay(10);*/

			LEFT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_REAR.brake();
            break;
        }
        pros::delay(2);
    }
	pros::delay(20);
}

void sortingTask() {
	int hue_value = 0;
	bool blueRock = false;
	bool redRock = false;
	bool noColor = true;

	OPTICAL_SENSOR.set_integration_time(40);
	OPTICAL_SENSOR.set_led_pwm(50);

	SORTING_MOTOR.set_zero_position(0);
	SORTING_MOTOR.tare_position();

	while (true) {
		/*RED ALLIANCE
		if(sorting_enable==true){
			hue_value = OPTICAL_SENSOR.get_hue();
			if(OPTICAL_SENSOR.get_proximity() > 230){
				if(hue_value > 215 && hue_value < 350){
					blueRock = true;
					redRock = false;
					noColor = false;
				}
				else if(hue_value < 6){
					redRock = true;
					blueRock = false;
					noColor = false;
				}
				else{
					redRock = false;
					blueRock = false;
					noColor = true;
				}
			}
			else{
				redRock = false;
				blueRock = false;
				noColor = true;
			}

			if (blueRock == true && redRock == false && noColor == false) {
				SORTING_MOTOR.move_absolute(50, 85);
				pros::delay(600);
				blueRock = false;
			}
			else if (redRock == true && blueRock == false && noColor == false) {
				SORTING_MOTOR.move_absolute(-50, 85);
				pros::delay(600);
				redRock = false;
			}
			else if (blueRock == false && redRock == false && noColor == true){
				SORTING_MOTOR.move_absolute(0, 150);
			}
			else{
				SORTING_MOTOR.move_absolute(0, 150);
			}
			if(OPTICAL_SENSOR.get_led_pwm()==0) OPTICAL_SENSOR.set_led_pwm(50);
			pros::delay(2);
		}*/

		/*BLUE ALLIANCE*/
		if(sorting_enable==true){
			hue_value = OPTICAL_SENSOR.get_hue();
			if(OPTICAL_SENSOR.get_proximity() > 235){
				if(hue_value > 215 && hue_value < 350){
					blueRock = true;
					redRock = false;
					noColor = false;
				}
				else if(hue_value < 10){
					redRock = true;
					blueRock = false;
					noColor = false;
				}
				else{
					redRock = false;
					blueRock = false;
					noColor = true;
				}
			}
			else{
				redRock = false;
				blueRock = false;
				noColor = true;
			}

			if (blueRock == true && redRock == false && noColor == false) {
				SORTING_MOTOR.move_absolute(-50, 90);
				pros::delay(700);
				blueRock = false;
				SORTING_MOTOR.move_absolute(0, 150);
			}
			else if (redRock == true && blueRock == false && noColor == false) {
				SORTING_MOTOR.move_absolute(50, 90);
				pros::delay(700);
				redRock = false;
				SORTING_MOTOR.move_absolute(0, 150);
			}
			else if (blueRock == false && redRock == false && noColor == true){
				SORTING_MOTOR.move_absolute(0, 150);
			}
			else{
				SORTING_MOTOR.move_absolute(0, 150);
			}
			if(OPTICAL_SENSOR.get_led_pwm()==0){
				OPTICAL_SENSOR.set_led_pwm(50);
				pros::delay(10);
			}
			pros::delay(10);
		}
		else{
			pros::delay(10);
			if(OPTICAL_SENSOR.get_led_pwm() == 50){
				OPTICAL_SENSOR.set_led_pwm(0);
				SORTING_MOTOR.move_absolute(0, 150);
				pros::delay(5);
			}
		}
	}
}

void rollerTask(){
	while(1){
		switch(Top_Roller_State){
			case 1:
				TOP_ROLLER_MOTOR.move(100);
				break;
			case -1:
				TOP_ROLLER_MOTOR.move(-100);
				break;
			case 0:
				TOP_ROLLER_MOTOR.move(0);
				break;
			default:
				break;
		}
		switch(Btm_Roller_State){
			case 1: 
				BTM_ROLLER_MOTOR.move(100);
				break;
			case -1: 
				BTM_ROLLER_MOTOR.move(-100);
				break;
			case 0: 
				BTM_ROLLER_MOTOR.move(0);
				break;
			default:
				break;
		}
		pros::delay(10);
		//printf("\n%lf", TOP_ROLLER_MOTOR.get_torque());
		//if(TOP_ROLLER_MOTOR.get_torque() >= 1.05 && TOP_ROLLER_MOTOR.get_actual_velocity() == 0){
		//	TOP_ROLLER_MOTOR.move(-95);
		//	pros::delay(500);
		//}
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	OPTICAL_SENSOR.disable_gesture();
	IMU_SENSOR.reset(true);
	IMU_SENSOR.set_data_rate(10);

	LEFT_MOTOR_FRONT.set_zero_position(0);
	LEFT_MOTOR_REAR.set_zero_position(0);
	RIGHT_MOTOR_FRONT.set_zero_position(0);
	RIGHT_MOTOR_REAR.set_zero_position(0);

	LEFT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_HOLD);
	LEFT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_HOLD);
	RIGHT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_HOLD);
	RIGHT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_HOLD);

	pros::Task sorting(sortingTask);
	pros::Task roller(rollerTask);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void moveAbsolute(double distance_cm, int vel){
	double target_distance = distance_cm/CM_PER_TICK;
	double offsetvel = 0.982;
	//double offsetvel = 0.981;
	LEFT_MOTOR_FRONT.tare_position();
	RIGHT_MOTOR_FRONT.tare_position();
	LEFT_MOTOR_REAR.tare_position();
	RIGHT_MOTOR_REAR.tare_position();
	LEFT_MOTOR_FRONT.set_zero_position(0);
	LEFT_MOTOR_REAR.set_zero_position(0);
	RIGHT_MOTOR_FRONT.set_zero_position(0);
	RIGHT_MOTOR_REAR.set_zero_position(0);
	LEFT_MOTOR_FRONT.move_absolute(target_distance,vel+offsetvel);
	RIGHT_MOTOR_FRONT.move_absolute(target_distance,vel);
	LEFT_MOTOR_REAR.move_absolute(target_distance,vel+offsetvel);
	RIGHT_MOTOR_REAR.move_absolute(target_distance,vel);
	while(1){
		if(fabs(LEFT_MOTOR_FRONT.get_position()) >= fabs(target_distance) || fabs(RIGHT_MOTOR_FRONT.get_position()) >= fabs(target_distance)){
			LEFT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_REAR.brake();
			while(LEFT_MOTOR_FRONT.get_actual_velocity() > 0 && RIGHT_MOTOR_FRONT.get_actual_velocity() > 0) pros::delay(2);
			break;
		}
		pros::delay(2);
	}
}

void autonomous() {
	//IMU_SENSOR.reset(true);
	//IMU_SENSOR.set_data_rate(10);
	//Top_Roller_State = 1;
	//Btm_Roller_State = 1;
	/*BLUE ALLIANCE*/
	/*COMPLEX ROUTE*/
	/*Top_Roller_State = 1;
	Btm_Roller_State = 1;
	moveAbsolute(300, 90);
	pros::delay(1000);
	turnPID(86);
	pros::delay(800);
	LEFT_MOTOR_FRONT.move(-80);
	LEFT_MOTOR_REAR.move(-80);
	RIGHT_MOTOR_FRONT.move(-80);
	RIGHT_MOTOR_REAR.move(-80);
	pros::delay(1000);
	LEFT_MOTOR_FRONT.move(0);
	LEFT_MOTOR_REAR.move(0);
	RIGHT_MOTOR_FRONT.move(0);
	RIGHT_MOTOR_REAR.move(0);
	pros::delay(800);
	moveAbsolute(35, 90);
	pros::delay(100);
	turnPID(81.5);
	moveAbsolute(255, 90);
	pros::delay(1000);
	turnPID(89.5);
	Top_Roller_State = 0;
	Btm_Roller_State = 0;
	pros::delay(100);
	LEFT_MOTOR_FRONT.move(-121);
	LEFT_MOTOR_REAR.move(-121);
	RIGHT_MOTOR_FRONT.move(-120);
	RIGHT_MOTOR_REAR.move(-120);
	pros::delay(7000);
	LEFT_MOTOR_FRONT.move(0);
	LEFT_MOTOR_REAR.move(0);
	RIGHT_MOTOR_FRONT.move(0);
	RIGHT_MOTOR_REAR.move(0);
	pros::delay(500);
	Top_Roller_State = 1;
	Btm_Roller_State = 1;
	moveAbsolute(30, 130);
	pros::delay(500);
	turnPID(76.5);
	LEFT_MOTOR_FRONT.move(-110);
	LEFT_MOTOR_REAR.move(-110);
	RIGHT_MOTOR_FRONT.move(-110);
	RIGHT_MOTOR_REAR.move(-110);
	pros::delay(2500);
	LEFT_MOTOR_FRONT.move(0);
	LEFT_MOTOR_REAR.move(0);
	RIGHT_MOTOR_FRONT.move(0);
	RIGHT_MOTOR_REAR.move(0);
	pros::delay(500);
	moveAbsolute(320, 130);
	pros::delay(1000);
	turnPID(73.5, false);
	pros::delay(1000);
	LEFT_MOTOR_FRONT.move(-90);
	LEFT_MOTOR_REAR.move(-90);
	RIGHT_MOTOR_FRONT.move(-90);
	RIGHT_MOTOR_REAR.move(-90);
	pros::delay(2000);
	LEFT_MOTOR_FRONT.move(0);
	LEFT_MOTOR_REAR.move(0);
	RIGHT_MOTOR_FRONT.move(0);
	RIGHT_MOTOR_REAR.move(0);
	pros::delay(500);
	moveAbsolute(15, 130);
	pros::delay(800);
	turnPID(81.5, false);
	pros::delay(1000);
	moveAbsolute(250, 130);
	pros::delay(500);
	turnPID(82);
	pros::delay(1000);
	turnPID(81);
	pros::delay(1000);
	moveAbsolute(170, 130);
	pros::delay(1000);
	turnPID(79, false);
	pros::delay(1000);
	EVAC_MOTOR.move(115);
	pros::delay(1100);
	LEFT_MOTOR_FRONT.move(-100);
	LEFT_MOTOR_REAR.move(-100);
	RIGHT_MOTOR_FRONT.move(-100);
	RIGHT_MOTOR_REAR.move(-100);
	pros::delay(2000);
	EVAC_MOTOR.move(-110);
	LEFT_MOTOR_FRONT.move(0);
	LEFT_MOTOR_REAR.move(0);
	RIGHT_MOTOR_FRONT.move(0);
	RIGHT_MOTOR_REAR.move(0);
	pros::delay(500);
	EVAC_MOTOR.move(0);
	Top_Roller_State = 0;
	Btm_Roller_State = 0;*/

	Top_Roller_State = 0;
	Btm_Roller_State = 0;
	LEFT_MOTOR_FRONT.move(-124);
	LEFT_MOTOR_REAR.move(-124);
	RIGHT_MOTOR_FRONT.move(-118);
	RIGHT_MOTOR_REAR.move(-118);
	pros::delay(6000);
	LEFT_MOTOR_FRONT.move(0);
	LEFT_MOTOR_REAR.move(0);
	RIGHT_MOTOR_FRONT.move(0);
	RIGHT_MOTOR_REAR.move(0);
	//Top_Roller_State = 1;
	//Btm_Roller_State = 1;
	pros::delay(900);
	moveAbsolute(20, 90);
	pros::delay(1000);
	turnPID(88.5);
	pros::delay(1000);
	LEFT_MOTOR_FRONT.move(-124);
	LEFT_MOTOR_REAR.move(-124);
	RIGHT_MOTOR_FRONT.move(-117);
	RIGHT_MOTOR_REAR.move(-118);
	pros::delay(3000);
	LEFT_MOTOR_FRONT.move(0);
	LEFT_MOTOR_REAR.move(0);
	RIGHT_MOTOR_FRONT.move(0);
	RIGHT_MOTOR_REAR.move(0);
	pros::delay(1000);
	/*SIMPLE SCORE
	*/
	//moveAbsolute(-210, -170);
	//turnPID(85.5);
	IMU_SENSOR.reset(true);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	int powerbtm = 90;
	int powertop = 90;
	int powerevac = 95;
	int powersorting = 50;
	bool tankdrive = false;
	IMU_SENSOR.tare();

	while (true) {
		//BTM_ROLLER_MOTOR.move((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)) * powerbtm);
		//TOP_ROLLER_MOTOR.move((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2)) * powertop);
		Top_Roller_State = (master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2));
		Btm_Roller_State = (master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2));
		EVAC_MOTOR.move((master.get_digital(DIGITAL_UP) - master.get_digital(DIGITAL_DOWN)) * powerevac);
		master.print(2,0,"%lf", IMU_SENSOR.get_yaw());
		if(master.get_digital_new_press(DIGITAL_B)) sorting_enable = !sorting_enable;

		if(master.get_digital_new_press(DIGITAL_Y)) autonomous();

		if(master.get_digital_new_press(DIGITAL_A)) tankdrive = !tankdrive;
		if(!tankdrive){
			int power = master.get_analog(ANALOG_LEFT_Y);
			int turn = master.get_analog(ANALOG_RIGHT_X);
			LEFT_MOTOR_FRONT.move(power*1.05 + turn);
			LEFT_MOTOR_REAR.move(power*1.05 + turn);
			RIGHT_MOTOR_FRONT.move(power - turn);
			RIGHT_MOTOR_REAR.move(power - turn);
		}
		else{
			int powerLeft = master.get_analog(ANALOG_LEFT_Y);
			int powerRight = master.get_analog(ANALOG_RIGHT_Y);
			LEFT_MOTOR_FRONT = powerLeft;
			LEFT_MOTOR_REAR = powerLeft;
			RIGHT_MOTOR_FRONT = powerRight;
			RIGHT_MOTOR_REAR = powerRight;
		}
		pros::delay(10);
	}
}