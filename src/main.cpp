#include "main.h"

const double FORWARD_KP = 1.9;
const double FORWARD_KI = 0.001;
const double FORWARD_KD = 0.00012;

const double TURN_KP = 4.1;
const double TURN_KI = 0.003;
const double TURN_KD = 0.001;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor LEFT_MOTOR_FRONT(LEFT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_FRONT(RIGHT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_REAR(RIGHT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LEFT_MOTOR_REAR(LEFT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor SORTING_MOTOR(SORTING_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BTM_ROLLER_MOTOR(BOTTOM_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor TOP_ROLLER_MOTOR(TOP_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Optical optical_sensor(OPTICAL_PORT, OPTICAL_PORT_UPDATE);
pros::c::optical_rgb_s_t rgb_value;

void forwardPID(double target_cm) {
    double errorLeft = 0, integralLeft = 0, derivativeLeft = 0, lastErrorLeft = 0, currentPositionLeft = 0;
	double forward_pid_outputL = 0;
	double errorRight = 0, integralRight = 0, derivativeRight = 0, lastErrorRight = 0, currentPositionRight = 0;
	double forward_pid_outputR = 0;
	double correctingLeft = 0, correctingRight = 0, deltaMotorError = 0;
	double powerLeft = 0, powerRight = 0;
	double correctingGain = 2.2;
	double targetLeft = fabs(target_cm) / CM_PER_TICK;
	double targetRight = fabs(target_cm) / CM_PER_TICK;
	bool forward = true;

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

		if(forward==true){
			LEFT_MOTOR_FRONT.move(powerLeft);
			LEFT_MOTOR_REAR.move(powerLeft);
			RIGHT_MOTOR_FRONT.move(powerRight);
			RIGHT_MOTOR_REAR.move(powerRight);
		}
		else{
			LEFT_MOTOR_FRONT.move(-powerLeft);
			LEFT_MOTOR_REAR.move(-powerLeft);
			RIGHT_MOTOR_FRONT.move(-powerRight);
			RIGHT_MOTOR_REAR.move(-powerRight);
		}
        lastErrorLeft = errorLeft;
		lastErrorRight = errorRight;

        if(fabs(errorLeft) <= 5 || fabs(errorRight) <= 5 || currentPositionLeft >= targetLeft || currentPositionRight >= targetRight){
			LEFT_MOTOR_FRONT.move(0);
			LEFT_MOTOR_REAR.move(0);
			RIGHT_MOTOR_FRONT.move(0);
			RIGHT_MOTOR_REAR.move(0);

			LEFT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_REAR.brake();
            break;
        }
        pros::delay(2);
    }
	target_cm = 0;
}

void turnPID(double target_degrees, bool turn_left = true) {
    double error = 0, integral = 0, derivative = 0, lastError = 0, currentRotation = 0;
	double turn_pid_output = 0;
	double target = fabs(target_degrees)*3.0;

	LEFT_MOTOR_FRONT.tare_position();

    while (1) {
        currentRotation = fabs(LEFT_MOTOR_FRONT.get_position());
        error = target - currentRotation;
        integral += error;
        derivative = error - lastError;

        turn_pid_output = ((TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative))*0.127;
		
		if(turn_left==true){
			LEFT_MOTOR_FRONT.move(-turn_pid_output);
			LEFT_MOTOR_REAR.move(-turn_pid_output);
			RIGHT_MOTOR_FRONT.move(turn_pid_output);
			RIGHT_MOTOR_REAR.move(turn_pid_output);
		}
		else{
			LEFT_MOTOR_FRONT.move(turn_pid_output);
			LEFT_MOTOR_REAR.move(turn_pid_output);
			RIGHT_MOTOR_FRONT.move(-turn_pid_output);
			RIGHT_MOTOR_REAR.move(-turn_pid_output);
		}
        lastError = error;

        if (fabs(error) <= 5 || currentRotation >= target) {
			LEFT_MOTOR_FRONT.move(0);
			LEFT_MOTOR_REAR.move(0);
			RIGHT_MOTOR_FRONT.move(0);
			RIGHT_MOTOR_REAR.move(0);

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
		if(optical_sensor.get_proximity() > 220){
			if(hue_value > 210){
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

		if (blueRock == true && redRock == false) {
			SORTING_MOTOR.move_absolute(50, 120);
			pros::delay(500);
		}
		else if (redRock == true && blueRock == false) {
			SORTING_MOTOR.move_absolute(-50, 120);
			pros::delay(500);
		}
		else if (blueRock == false && redRock == false && noColor == true){
			SORTING_MOTOR.move_absolute(0, 140);
			pros::delay(100);
		}

		if(optical_sensor.get_led_pwm() == 0){
			optical_sensor.set_led_pwm(50);
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
	forwardPID(40);
	pros::delay(1000);
	turnPID(90);
	pros::delay(2);
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
	int powerbtm = 120;
	int powertop = 120;
	bool tankdrive = false;

	master.print(2, 0, "%s", "A:tankdrive");

	while (true) {
		BTM_ROLLER_MOTOR.move((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)) * powerbtm);
		TOP_ROLLER_MOTOR.move((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2)) * powertop);

		if (master.get_digital_new_press(DIGITAL_DOWN)){
			SORTING_MOTOR.tare_position();
			SORTING_MOTOR.set_zero_position(0);
			pros::delay(2);
		}

		if(master.get_digital_new_press(DIGITAL_A)) tankdrive = !tankdrive;

		if(!tankdrive){
			int power = master.get_analog(ANALOG_LEFT_Y);
			int turn = master.get_analog(ANALOG_RIGHT_X);
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