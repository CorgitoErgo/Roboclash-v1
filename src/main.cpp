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
pros::Optical optical_sensor(OPTICAL_PORT, OPTICAL_PORT_UPDATE);
pros::Imu IMU_SENSOR(IMU_SENSOR_PORT);
pros::c::optical_rgb_s_t rgb_value;

void forwardPID(double target_cm) {
	double currentPositionLeft = 0, currentPositionRight = 0;
	double error = 0, integral = 0, derivative = 0, lastError = 0, power = 0, currentPosition = 0;
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
        currentPositionLeft = (fabs(LEFT_MOTOR_FRONT.get_position()) + fabs(LEFT_MOTOR_REAR.get_position()))/2;
		currentPositionRight = (fabs(RIGHT_MOTOR_FRONT.get_position()) + fabs(RIGHT_MOTOR_REAR.get_position()))/2;
		currentPosition = (currentPositionLeft + currentPositionRight)/2;
		correctingPower = IMU_SENSOR.get_rotation();

		error = target - currentPosition;

		integral += error;

		derivative = error - lastError;

		power = ((FORWARD_KP * error) + (FORWARD_KI * integral) + (FORWARD_KD * derivative));

		if(forward==true){
			LEFT_MOTOR_FRONT.move_velocity(power+correctingPower);
			LEFT_MOTOR_REAR.move_velocity(power+correctingPower);
			RIGHT_MOTOR_FRONT.move_velocity(power-correctingPower);
			RIGHT_MOTOR_REAR.move_velocity(power-correctingPower);
		}
		else{
			LEFT_MOTOR_FRONT.move_velocity(-(power+correctingPower));
			LEFT_MOTOR_REAR.move_velocity(-(power+correctingPower));
			RIGHT_MOTOR_FRONT.move_velocity(-(power-correctingPower));
			RIGHT_MOTOR_REAR.move_velocity(-(power-correctingPower));
		}
		lastError = error;

		if(fabs(error) <= 3 || (currentPositionLeft >= target && currentPositionRight >= target)){
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
		}
        pros::delay(2);
    }
	master.print(0,0,"%lf", currentPositionLeft*CM_PER_TICK);
	pros::delay(50);
	master.print(1,0,"%lf", currentPositionRight*CM_PER_TICK);
	target_cm = 0;
	pros::delay(100);
}

void turnPID(double target_degrees, bool turn_left = true) {
    double error = 0, integral = 0, derivative = 0, lastError = 0, currentRotation = 0;
	double currentHeading = 0;
	double power = 0;
	double target = target_degrees;

	LEFT_MOTOR_FRONT.tare_position();
	RIGHT_MOTOR_FRONT.tare_position();
	LEFT_MOTOR_REAR.tare_position();
	RIGHT_MOTOR_REAR.tare_position();
	IMU_SENSOR.tare();
	imu_error = IMU_SENSOR.get_rotation();

    while (1) {
		currentHeading = fabs(IMU_SENSOR.get_rotation());
        error = target - currentHeading - imu_error;
        integral += error;
        derivative = error - lastError;

        power = (TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative);
		
		if(turn_left==true){
			LEFT_MOTOR_FRONT.move_velocity(-power);
			LEFT_MOTOR_REAR.move_velocity(-power);
			RIGHT_MOTOR_FRONT.move_velocity(power);
			RIGHT_MOTOR_REAR.move_velocity(power);
		}
		else{
			LEFT_MOTOR_FRONT.move_velocity(power);
			LEFT_MOTOR_REAR.move_velocity(power);
			RIGHT_MOTOR_FRONT.move_velocity(-power);
			RIGHT_MOTOR_REAR.move_velocity(-power);
		}
        lastError = error;

        if ((fabs(error) <= 0.01) || currentHeading >= target) {
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
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_REAR.brake();
            break;
        }
        pros::delay(10);
    }
	master.print(2,0,"%lf", currentHeading);
	target_degrees = 0;
	pros::delay(100);
}

void sortingTask() {
	int hue_value = 0;
	bool blueRock = false;
	bool redRock = false;
	bool noColor = true;

	optical_sensor.set_integration_time(40);
	optical_sensor.set_led_pwm(50);

	SORTING_MOTOR.set_zero_position(0);
	SORTING_MOTOR.tare_position();

	while (true) {
		/*RED ALLIANCE
		if(sorting_enable==true){
			hue_value = optical_sensor.get_hue();
			if(optical_sensor.get_proximity() > 230){
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
			if(optical_sensor.get_led_pwm()==0) optical_sensor.set_led_pwm(50);
			pros::delay(2);
		}*/

		/*BLUE ALLIANCE*/
		if(sorting_enable==true){
			hue_value = optical_sensor.get_hue();
			if(optical_sensor.get_proximity() > 230){
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
				SORTING_MOTOR.move_absolute(-50, 85);
				pros::delay(600);
				blueRock = false;
			}
			else if (redRock == true && blueRock == false && noColor == false) {
				SORTING_MOTOR.move_absolute(50, 85);
				pros::delay(600);
				redRock = false;
			}
			else if (blueRock == false && redRock == false && noColor == true){
				SORTING_MOTOR.move_absolute(0, 150);
			}
			else{
				SORTING_MOTOR.move_absolute(0, 150);
			}
			if(optical_sensor.get_led_pwm()==0) optical_sensor.set_led_pwm(50);
			pros::delay(2);
		}
		else{
			pros::delay(2);
			if(optical_sensor.get_led_pwm() == 50){
				optical_sensor.set_led_pwm(0);
				SORTING_MOTOR.move_absolute(0, 150);
			}
		}
	}
}

void rollerTask(){
	while(1){
		switch(Top_Roller_State){
			case 1:
				TOP_ROLLER_MOTOR.move(95);
				break;
			case -1:
				TOP_ROLLER_MOTOR.move(-95);
				break;
			case 0:
				TOP_ROLLER_MOTOR.move(0);
				break;
			default:
				break;
		}
		switch(Btm_Roller_State){
			case 1: 
				BTM_ROLLER_MOTOR.move(95);
				break;
			case -1: 
				BTM_ROLLER_MOTOR.move(-95);
				break;
			case 0: 
				BTM_ROLLER_MOTOR.move(0);
				break;
			default:
				break;
		}
		pros::delay(10);
		//printf("\n%lf", TOP_ROLLER_MOTOR.get_torque());
		if(TOP_ROLLER_MOTOR.get_torque() >= 1.05 && TOP_ROLLER_MOTOR.get_actual_velocity() == 0){
			TOP_ROLLER_MOTOR.move(-95);
			pros::delay(800);
		}
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	optical_sensor.disable_gesture();
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
	master.clear();
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

void autonomous() {
	IMU_SENSOR.reset(true);
	IMU_SENSOR.set_data_rate(10);
	Top_Roller_State = 1;
	Btm_Roller_State = 1;
	/*forwardPID(400);
	pros::delay(5);
	turnPID(90);
	pros::delay(5);
	forwardPID(20);
	pros::delay(5);
	turnPID(90);
	pros::delay(5);
	forwardPID(400);
	pros::delay(5);
	turnPID(90, false);
	pros::delay(5);
	forwardPID(20);
	pros::delay(5);
	turnPID(90, false);
	pros::delay(5);
	forwardPID(350);
	pros::delay(5);
	turnPID(90);
	pros::delay(5);
	forwardPID(150);
	pros::delay(5);
	turnPID(90);
	pros::delay(5);
	forwardPID(300);
	pros::delay(5);*/
	turnPID(87.5);
	turnPID(87.5);
	turnPID(87.5);
	turnPID(87.5);
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

	while (true) {
		//BTM_ROLLER_MOTOR.move((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)) * powerbtm);
		//TOP_ROLLER_MOTOR.move((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2)) * powertop);
		Top_Roller_State = (master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2));
		Btm_Roller_State = (master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2));
		EVAC_MOTOR.move((master.get_digital(DIGITAL_UP) - master.get_digital(DIGITAL_DOWN)) * powerevac);

		if(master.get_digital_new_press(DIGITAL_B)) sorting_enable = !sorting_enable;

		if(master.get_digital_new_press(DIGITAL_Y)) autonomous();

		if(master.get_digital_new_press(DIGITAL_A)) tankdrive = !tankdrive;
		if(!tankdrive){
			int power = master.get_analog(ANALOG_LEFT_Y);
			int turn = master.get_analog(ANALOG_RIGHT_X);
			LEFT_MOTOR_FRONT.move(power + turn);
			LEFT_MOTOR_REAR.move(power + turn);
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