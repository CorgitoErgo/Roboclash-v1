#include "main.h"

/*Check definitions header file for port assignments*/
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor LEFT_MOTOR_FRONT(LEFT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_FRONT(RIGHT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_REAR(RIGHT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LEFT_MOTOR_REAR(LEFT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor SORTING_MOTOR(SORTING_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BTM_ROLLER_MOTOR(BOTTOM_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor TOP_ROLLER_MOTOR(TOP_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Optical optical_sensor(OPTICAL_PORT, OPTICAL_PORT_UPDATE);
pros::Imu IMU_SENSOR(IMU_SENSOR_PORT);
pros::c::optical_rgb_s_t rgb_value;

void forwardPID(double target_cm) {
    double errorLeft = 0, integralLeft = 0, derivativeLeft = 0, lastErrorLeft = 0, currentPositionLeft = 0;
	double forward_pid_outputL = 0;
	double errorRight = 0, integralRight = 0, derivativeRight = 0, lastErrorRight = 0, currentPositionRight = 0;
	double forward_pid_outputR = 0;
	double correctingLeft = 0, correctingRight = 0, deltaMotorError = 0;
	double powerLeft = 0, powerRight = 0;
	double correctingGain = 2.1;
	double offsetPower = 3;
	double targetLeft = fabs(target_cm) / CM_PER_TICK;
	double targetRight = fabs(target_cm) / CM_PER_TICK;
	bool forward = true;
	bool leftMotorRun = true;
	bool rightMotorRun = true;

	LEFT_MOTOR_FRONT.tare_position();
	RIGHT_MOTOR_FRONT.tare_position();

	if(target_cm<0) forward = false;

    while (1) {
        currentPositionLeft = fabs(LEFT_MOTOR_FRONT.get_position());
		currentPositionRight = fabs(RIGHT_MOTOR_FRONT.get_position());
		deltaMotorError = fabs(currentPositionLeft - currentPositionRight);

		if(currentPositionLeft > currentPositionRight){
			correctingRight = deltaMotorError * correctingGain;
			correctingLeft = -(deltaMotorError * correctingGain);
		}
		else if(currentPositionLeft < currentPositionRight){
			correctingLeft = deltaMotorError * correctingGain;
			correctingRight = -(deltaMotorError * correctingGain);
		}
		else{
			correctingLeft = 0;
			correctingRight = 0;
		}
        errorLeft = targetLeft - currentPositionLeft;
		errorRight = targetRight - currentPositionRight;

        integralLeft += errorLeft;
		integralRight += errorRight;

        derivativeLeft = errorLeft - lastErrorLeft;
        derivativeRight = errorRight - lastErrorRight;

        forward_pid_outputL = (FORWARD_KP * errorLeft) + (FORWARD_KI * integralLeft) + (FORWARD_KD * derivativeLeft);
		forward_pid_outputR = (FORWARD_KP * errorRight) + (FORWARD_KI * integralRight) + (FORWARD_KD * derivativeRight);
		
		powerLeft = (forward_pid_outputL+correctingLeft)*0.127;
		powerRight = (forward_pid_outputR+correctingRight)*0.127;
		printf("\npowLeft:%lf", powerLeft);
		printf("\npowRight:%lf", powerRight);

		if(forward==true){
			if(leftMotorRun == true){
				LEFT_MOTOR_FRONT.move(powerLeft);
				LEFT_MOTOR_REAR.move(powerLeft);
			}
			else{
				LEFT_MOTOR_FRONT.move(-1);
				LEFT_MOTOR_REAR.move(-1);
			}
			if(rightMotorRun == true){
				RIGHT_MOTOR_FRONT.move(powerRight+offsetPower);
				RIGHT_MOTOR_REAR.move(powerRight+offsetPower);
			}
			else{
				RIGHT_MOTOR_FRONT.move(-1);
				RIGHT_MOTOR_REAR.move(-1);
			}
		}
		else{
			if(leftMotorRun == true){
				LEFT_MOTOR_FRONT.move(-powerLeft);
				LEFT_MOTOR_REAR.move(-powerLeft);
			}
			else{
				LEFT_MOTOR_FRONT.move(1);
				LEFT_MOTOR_REAR.move(1);
			}
			if(rightMotorRun == true){
				RIGHT_MOTOR_FRONT.move(-powerRight-offsetPower);
				RIGHT_MOTOR_REAR.move(-powerRight-offsetPower);
			}
			else{
				RIGHT_MOTOR_FRONT.move(1);
				RIGHT_MOTOR_REAR.move(1);
			}
		}
        lastErrorLeft = errorLeft;
		lastErrorRight = errorRight;

		if(fabs(errorLeft) <= 1.5){
			LEFT_MOTOR_FRONT.move(0);
			LEFT_MOTOR_REAR.move(0);
			LEFT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			leftMotorRun = false;
		}
		if(fabs(errorRight) <= 1.5){
			RIGHT_MOTOR_FRONT.move(0);
			RIGHT_MOTOR_REAR.move(0);
			RIGHT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_REAR.brake();
			rightMotorRun = false;
		}
		if(rightMotorRun == false && leftMotorRun == false){
			break;
		}
        /*if(fabs(errorLeft) <= 3 || fabs(errorRight) <= 3 || currentPositionLeft >= targetLeft || currentPositionRight >= targetRight){
			if(forward==true){
				LEFT_MOTOR_FRONT.move(-2);
				LEFT_MOTOR_REAR.move(-2);
				RIGHT_MOTOR_FRONT.move(-2);
				RIGHT_MOTOR_REAR.move(-2);
			}
			else{
				LEFT_MOTOR_FRONT.move(2);
				LEFT_MOTOR_REAR.move(2);
				RIGHT_MOTOR_FRONT.move(2);
				RIGHT_MOTOR_REAR.move(2);
			}
			pros::delay(10);
			
			LEFT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_REAR.brake();
            break;
        }*/
        pros::delay(2);
    }
	target_cm = 0;
}

void turnPID(double target_degrees, bool turn_left = true) {
    double error = 0, integral = 0, derivative = 0, lastError = 0, currentRotation = 0;
	double errorHeading = 0, currentHeading = 0;
	double turn_pid_output = 0;
	double target = fabs(target_degrees);
	double offsetPower = 3;

	LEFT_MOTOR_FRONT.tare_position();
	RIGHT_MOTOR_FRONT.tare_position();
	IMU_SENSOR.tare_rotation();

    while (1) {
        currentRotation = (fabs(LEFT_MOTOR_FRONT.get_position()) + fabs(RIGHT_MOTOR_FRONT.get_position()))/2;
		currentHeading = fabs(IMU_SENSOR.get_rotation());
        error = target - currentHeading;
		errorHeading = target - currentHeading;
        integral += error;
        derivative = error - lastError;

        turn_pid_output = ((TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative))*0.127;
		printf("\nturn error:%lf", error);
		
		if(turn_left==true){
			LEFT_MOTOR_FRONT.move(-turn_pid_output);
			LEFT_MOTOR_REAR.move(-turn_pid_output);
			RIGHT_MOTOR_FRONT.move(turn_pid_output+offsetPower);
			RIGHT_MOTOR_REAR.move(turn_pid_output+offsetPower);
		}
		else{
			LEFT_MOTOR_FRONT.move(turn_pid_output);
			LEFT_MOTOR_REAR.move(turn_pid_output);
			RIGHT_MOTOR_FRONT.move(-turn_pid_output-offsetPower);
			RIGHT_MOTOR_REAR.move(-turn_pid_output-offsetPower);
		}
        lastError = error;
		printf("heading:%lf", currentHeading);
		printf("rotation:%lf", currentRotation);

        if ((fabs(error) <= 2)) {
			if(turn_left){
				LEFT_MOTOR_FRONT.move(-2);
				LEFT_MOTOR_REAR.move(-2);
				RIGHT_MOTOR_FRONT.move(2);
				RIGHT_MOTOR_REAR.move(2);
			}
			else{
				LEFT_MOTOR_FRONT.move(2);
				LEFT_MOTOR_REAR.move(2);
				RIGHT_MOTOR_FRONT.move(-2);
				RIGHT_MOTOR_REAR.move(-2);
			}
			pros::delay(10);

			LEFT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_REAR.brake();
            break;
        }
        pros::delay(2);
    }
	target_degrees = 0;
}

void sortingMotor() {
	int hue_value = 0;
	bool blueRock = false;
	bool redRock = false;
	bool noColor = true;

	optical_sensor.set_integration_time(40); 	//40ms intervals
	optical_sensor.set_led_pwm(50); 			//50% brightness

	SORTING_MOTOR.set_zero_position(0);
	SORTING_MOTOR.tare_position();

	while (true) {
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

		if(optical_sensor.get_led_pwm() == 0){
			optical_sensor.set_led_pwm(50);
		}
		pros::delay(2);
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
	IMU_SENSOR.set_data_rate(2);

	LEFT_MOTOR_FRONT.set_zero_position(0);
	LEFT_MOTOR_REAR.set_zero_position(0);
	RIGHT_MOTOR_FRONT.set_zero_position(0);
	RIGHT_MOTOR_REAR.set_zero_position(0);

	LEFT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_BRAKE);
	LEFT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_BRAKE);
	RIGHT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_BRAKE);
	RIGHT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_BRAKE);

	pros::Task sorting(sortingMotor);
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
	TOP_ROLLER_MOTOR.move(100);
	BTM_ROLLER_MOTOR.move(100);
	forwardPID(30);
	pros::delay(5);
	turnPID(90);
	pros::delay(5);
	forwardPID(20);
	pros::delay(5);
	turnPID(90);
	pros::delay(5);
	forwardPID(30);
	pros::delay(5);
	turnPID(90);
	pros::delay(5);
	forwardPID(20);
	pros::delay(5);
	turnPID(90, false);
	pros::delay(5);
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
	int powerbtm = 95;
	int powertop = 95;
	bool tankdrive = false;

	master.print(2, 0, "%s", "A:tankdrive");

	while (true) {
		BTM_ROLLER_MOTOR.move((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)) * powerbtm);
		TOP_ROLLER_MOTOR.move((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2)) * powertop);

		if(master.get_digital_new_press(DIGITAL_DOWN)){
			SORTING_MOTOR.tare_position();
			SORTING_MOTOR.set_zero_position(0);
			pros::delay(2);
		}

		if(master.get_digital_new_press(DIGITAL_UP)) autonomous();

		if(master.get_digital_new_press(DIGITAL_A)) tankdrive = !tankdrive;

		if(!tankdrive){
			int power = master.get_analog(ANALOG_LEFT_Y)*0.97;
			int turn = master.get_analog(ANALOG_RIGHT_X)*0.9;
			LEFT_MOTOR_FRONT = power + turn;
			LEFT_MOTOR_REAR = power + turn;
			RIGHT_MOTOR_FRONT = power - turn;
			RIGHT_MOTOR_REAR = power - turn;
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