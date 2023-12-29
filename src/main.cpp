#include "main.h"
#include "definitions.h"

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
    double errorL = 0, integralL = 0, derivativeL = 0, lastErrorL = 0, currentPositionL = 0;
	double errorR = 0, integralR = 0, derivativeR = 0, lastErrorR = 0, currentPositionR = 0;
	double correctingL = 0;
	double correctingR = 0;
	double correctingGain = 2.2;
	double deltaMotorError = 0;
	LEFT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_BRAKE);
	RIGHT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_BRAKE);
	LEFT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_BRAKE);
	RIGHT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_BRAKE);
	LEFT_MOTOR_FRONT.tare_position();
	RIGHT_MOTOR_FRONT.tare_position();
	bool forward = true;
	if(target_cm<0) forward = false;
	double targetL = fabs(target_cm) / CM_PER_TICK;
	double targetR = fabs(target_cm) / CM_PER_TICK;
    while (1) {
        currentPositionL = fabs(LEFT_MOTOR_FRONT.get_position());
		currentPositionR = fabs(RIGHT_MOTOR_FRONT.get_position());
		deltaMotorError = fabs(currentPositionL - currentPositionR);
		if(currentPositionL > currentPositionR){
			correctingR = deltaMotorError * correctingGain;
			correctingL = -deltaMotorError * correctingGain;
		}
		else if(currentPositionR > currentPositionL){
			correctingL = deltaMotorError * correctingGain;
			correctingR = -deltaMotorError * correctingGain;
		}
		else{
			correctingL = 0;
			correctingR = 0;
		}

        errorL = targetL - currentPositionL;
		errorR = targetR - currentPositionR;

        integralL += errorL;
		integralR += errorR;

        derivativeL = errorL - lastErrorL;
        derivativeR = errorR - lastErrorR;

        forward_pid_outputL = (FORWARD_KP * errorL) + (FORWARD_KI * integralL) + (FORWARD_KD * derivativeL);
		forward_pid_outputR = (FORWARD_KP * errorR) + (FORWARD_KI * integralR) + (FORWARD_KD * derivativeR);
        // Use the output to set motor power
        /* Set motor power based on output */
		if(forward==true){
			LEFT_MOTOR_FRONT.move((forward_pid_outputL+correctingL)*0.127);
			RIGHT_MOTOR_FRONT.move((forward_pid_outputR+correctingR)*0.127);
			LEFT_MOTOR_REAR.move((forward_pid_outputL+correctingL)*0.127);
			RIGHT_MOTOR_REAR.move((forward_pid_outputR+correctingR)*0.127);
			//LEFT_MOTOR_FRONT.move((forward_pid_outputL+deltaMotorError));
			//RIGHT_MOTOR_FRONT.move((forward_pid_outputL-deltaMotorError));
		}
		else{
			LEFT_MOTOR_FRONT.move(-(forward_pid_outputL+correctingL)*0.127);
			RIGHT_MOTOR_FRONT.move(-(forward_pid_outputL+correctingR)*0.127);
			LEFT_MOTOR_REAR.move((forward_pid_outputL+correctingL)*0.127);
			RIGHT_MOTOR_REAR.move((forward_pid_outputR+correctingR)*0.127);
			//LEFT_MOTOR_FRONT.move(-(forward_pid_outputL+deltaMotorError));
			//RIGHT_MOTOR_FRONT.move(-(forward_pid_outputR-deltaMotorError));
		}
		//deltaMotorError = TURN_TRACKING_WHEEL.get_value();
        lastErrorL = errorL;
		lastErrorR = errorR;

        if (fabs(errorL) <= 5 || currentPositionL >= targetL || fabs(errorR) <= 5 || currentPositionR >= targetR) {
			LEFT_MOTOR_FRONT.move(0);
			RIGHT_MOTOR_FRONT.move(0);
			LEFT_MOTOR_REAR.move(0);
			RIGHT_MOTOR_REAR.move(0);

			LEFT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_REAR.brake();
            break;  // Exit the loop if close to the target
        }
		//master.print(2, 0, "%lf", forward_pid_output*0.127);
        pros::delay(2);  // Adjust the delay based on your control loop speed
    }
	//LEFT_MOTOR_FRONT.brake();
	//RIGHT_MOTOR_FRONT.brake();
	//LEFT_MOTOR_FRONT.tare_position();
	//RIGHT_MOTOR_FRONT.tare_position();
	//LEFT_MOTOR_FRONT.move_absolute(0, 100);
	//RIGHT_MOTOR_FRONT.move_absolute(0, 100);
	target_cm = 0;
	//LEFT_MOTOR_FRONT.tare_position();
	//RIGHT_MOTOR_FRONT.tare_position();
	//LEFT_TRACKING_WHEEL.reset();
}

void turnPID(double target_degrees, bool turn_left = true) {
    double error = 0, integral = 0, derivative = 0, lastError = 0, currentRotation = 0;
	//INERTIA_SENSOR.tare_rotation();
	//INERTIA_SENSOR.tare();
	LEFT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_BRAKE);
	RIGHT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_BRAKE);
	LEFT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_BRAKE);
	RIGHT_MOTOR_REAR.set_brake_mode(MOTOR_BRAKE_BRAKE);
	LEFT_MOTOR_FRONT.set_zero_position(0);
	LEFT_MOTOR_REAR.set_zero_position(0);
	RIGHT_MOTOR_FRONT.set_zero_position(0);
	RIGHT_MOTOR_REAR.set_zero_position(0);
	LEFT_MOTOR_FRONT.tare_position();
	LEFT_MOTOR_REAR.tare_position();
	RIGHT_MOTOR_FRONT.tare_position();
	RIGHT_MOTOR_REAR.tare_position();
	double target = fabs(target_degrees)*3.0;
	//double yaw_error = 5;
    while (1) {
        currentRotation = fabs(LEFT_MOTOR_FRONT.get_position());
        error = target - currentRotation;
        integral += error;
        derivative = error - lastError;

        turn_pid_output = (TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative);
		
        // Use the output to set motor power for turning
        /* Set motor power for turning based on output */
		if(turn_left==true){
			LEFT_MOTOR_FRONT.move(-turn_pid_output*0.127);
			RIGHT_MOTOR_FRONT.move(turn_pid_output*0.127);
			LEFT_MOTOR_REAR.move(-turn_pid_output*0.127);
			RIGHT_MOTOR_REAR.move(turn_pid_output*0.127);
		}
		else{
			LEFT_MOTOR_FRONT.move(turn_pid_output*0.127);
			RIGHT_MOTOR_FRONT.move(-turn_pid_output*0.127);
			LEFT_MOTOR_REAR.move(-turn_pid_output*0.127);
			RIGHT_MOTOR_REAR.move(turn_pid_output*0.127);
		}

        lastError = error;

        if (fabs(error) <= 5 || currentRotation >= target) {
			LEFT_MOTOR_FRONT.move(0);
			RIGHT_MOTOR_FRONT.move(0);
			LEFT_MOTOR_REAR.move(0);
			RIGHT_MOTOR_REAR.move(0);

			LEFT_MOTOR_FRONT.brake();
			RIGHT_MOTOR_FRONT.brake();
			LEFT_MOTOR_REAR.brake();
			RIGHT_MOTOR_REAR.brake();
            break;  // Exit the loop if close to the target
        }

        pros::delay(2);  // Adjust the delay based on your control loop speed
    }
	//LEFT_MOTOR_FRONT.brake();
	//RIGHT_MOTOR_FRONT.brake();
	//LEFT_MOTOR_FRONT.tare_position();
	//RIGHT_MOTOR_FRONT.tare_position();
	//LEFT_MOTOR_FRONT.move_absolute(0, 100);
	//RIGHT_MOTOR_FRONT.move_absolute(0, 100);
	target_degrees = 0;
	//INERTIA_SENSOR.tare_rotation();
	//LEFT_MOTOR_FRONT.tare_position();
	//RIGHT_MOTOR_FRONT.tare_position();
}


double errorLeft = 0;
double errorRight = 0;
double prevErrorLeft = 0;
double prevErrorRight = 0;
double deltaErrorLeft = 0;
double deltaErrorRight = 0;
double trackLeft = 0;
double trackRight = 0;
double powerL = 0;
double powerR = 0;
double target_distance_l = 20;
double target_distance_r = 20;

bool left = false;
bool right = false;
bool blueRock = false;
bool redRock = false;
bool noneColor = true;

void sortingMotor() {
	//pros::Motor motor1(Motor_Port, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	SORTING_MOTOR.set_zero_position(0);
	SORTING_MOTOR.tare_position();
	SORTING_MOTOR.set_brake_mode(MOTOR_BRAKE_BRAKE);
	while (true) {
		if (blueRock == true && redRock == false) {
			SORTING_MOTOR.move_absolute(50, 120);
			pros::delay(500);
			//SORTING_MOTOR.move_absolute(0, 130);
		}
		else if (redRock == true && blueRock == false) {
			SORTING_MOTOR.move_absolute(-50, 120);
			pros::delay(500);
			//SORTING_MOTOR.move_absolute(0, 130);
		}
		else if (blueRock == false && redRock == false && noneColor == true){
			SORTING_MOTOR.move_absolute(0, 140);
			pros::delay(80);
			//while(motor1.is_stopped()==0) pros::delay(2);
			//SORTING_MOTOR.brake();
		}
	}
}

/*
void pidmove(){
	LEFT_TRACKING_WHEEL.reset();
	RIGHT_TRACKING_WHEEL.reset();
	LEFT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_HOLD);
	RIGHT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_HOLD);
	LEFT_MOTOR_FRONT.set_zero_position(0);
	RIGHT_MOTOR_FRONT.set_zero_position(0);
	double target_distance_left = target_distance_l;
	double target_distance_right = target_distance_r;
	double encoderError = 0;
	INERTIA_SENSOR.tare_yaw();
	double heading = 0;
	errorLeft = target_distance_l;
	errorRight = target_distance_r;
	while(fabs(errorLeft)>=5){
		//LEFT_MOTOR_FRONT.set_zero_position(0);
		//RIGHT_MOTOR_FRONT.set_zero_position(0);
		trackLeft = abs(LEFT_TRACKING_WHEEL.get_value());
		trackRight = abs(RIGHT_TRACKING_WHEEL.get_value());
		heading = INERTIA_SENSOR.get_yaw();
		encoderError = trackLeft - trackRight;
		if(trackLeft > trackRight){
			trackLeft += encoderError;
			trackRight -= encoderError;
		}
		if(trackRight > trackLeft){
			trackRight += encoderError;
			trackLeft -= encoderError;
		}
		errorLeft = target_distance_left*TRACKING_TICKS_PER_CM - trackLeft;
        errorRight = target_distance_right*TRACKING_TICKS_PER_CM - trackRight;
		if(heading>0){
			errorLeft -= heading;
			errorRight += heading;
		}
		if(heading<0){
			errorRight -= heading;
			errorLeft += heading;
		}

		deltaErrorLeft = errorLeft - prevErrorLeft;
        deltaErrorRight = errorRight - prevErrorRight;

		powerL = base_kp * errorLeft + base_kd * deltaErrorLeft; // divide 25 to prevent from going insane - idk why
        powerR = base_kp * errorRight + base_kd * deltaErrorRight;

		LEFT_MOTOR_FRONT.move(powerL);
		RIGHT_MOTOR_FRONT.move(powerR);

		prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;
		//printf("PowL:%i,PowR:%i\nErrorL:%i,ErrorR:%i",powerL,powerR,errorLeft,errorRight);
		pros::delay(2);
	}
	LEFT_MOTOR_FRONT.brake();
	RIGHT_MOTOR_FRONT.brake();
	LEFT_TRACKING_WHEEL.reset();
	RIGHT_TRACKING_WHEEL.reset();
	pros::delay(2);
	target_distance_l = 0;
	target_distance_r = 0;
}*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");

	//pros::lcd::register_btn1_cb(on_center_button);
	//pros::Imu INERTIA_SENSOR(INERTIA_SENSOR_PORT);
  	//INERTIA_SENSOR.reset()==0;
	optical_sensor.disable_gesture();
	pros::Task sorting(sortingMotor);
	LEFT_MOTOR_FRONT.set_zero_position(0);
	LEFT_MOTOR_REAR.set_zero_position(0);
	RIGHT_MOTOR_FRONT.set_zero_position(0);
	RIGHT_MOTOR_REAR.set_zero_position(0);
	//LEFT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_COAST);
	//RIGHT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_COAST);
	//LEFT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_HOLD);
	//RIGHT_MOTOR_FRONT.set_brake_mode(MOTOR_BRAKE_HOLD);
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
	pros::delay(80);
	//turnPID(90, true);
	//pros::delay(50);
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	optical_sensor.set_integration_time(40);
	//pros::Motor LEFT_MOTOR_FRONT(LEFT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	//pros::Motor RIGHT_MOTOR_FRONT(RIGHT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	//pros::Imu INERTIA_SENSOR(INERTIA_SENSOR_PORT);
	//pros::ADIEncoder LEFT_TRACKING_WHEEL(LEFT_TRACKING_PORT_TOP, LEFT_TRACKING_PORT_BTM, false);
	//pros::ADIEncoder RIGHT_TRACKING_WHEEL(RIGHT_TRACKING_PORT_TOP, RIGHT_TRACKING_PORT_BTM, true);
	//vision_sensor.clear_led();
	//if(master.get_digital_new_press(DIGITAL_A)) INERTIA_SENSOR.reset(true);
	//while(INERTIA_SENSOR.is_calibrating()) pros::c::delay(5);
	int timer = 0;
	bool toggle_led = true;
	int ledPWM = 100;
	float redError=0;
	float greenError=0;
	float BlueError=0;
	int hue_value=0;
	bool tankdrive = false;
	bool start_bottom_roller = false;
	bool start_top_roller = false;
	int powerbtm = 120;
	int powertop = 120;
	master.clear();
	optical_sensor.set_led_pwm(ledPWM);

	while (true) {
		//pros::c::imu_accel_s_t accel = INERTIA_SENSOR.get_accel();
		//rgb_value = optical_sensor.get_rgb();
		hue_value = optical_sensor.get_hue();
		if(optical_sensor.get_led_pwm() == 0){
			optical_sensor.set_led_pwm(ledPWM);
		}
		
		/*if(master.get_digital_new_press(DIGITAL_R1)) toggle_led = !toggle_led;
		if(master.get_digital_new_press(DIGITAL_R2)) ledPWM += 20;
		if(ledPWM > 100){
			ledPWM = 20;
			master.clear();
		}
		if(toggle_led) optical_sensor.set_led_pwm(ledPWM);
		else optical_sensor.set_led_pwm(0);*/
		//vision_sensor.read_by_sig(0, 1, 1, object_arr);
		/*if(master.get_digital_new_press(DIGITAL_A)){
			INERTIA_SENSOR.tare();
			RIGHT_TRACKING_WHEEL.reset();
			LEFT_TRACKING_WHEEL.reset();
			master.clear();
			pros::delay(50);
		}*/
		//while(INERTIA_SENSOR.is_calibrating()) pros::c::delay(5);
		/*if(master.get_digital_new_press(DIGITAL_B)){
			page++;
			master.clear();
			if(page>3) page = 0;
		}*/
		/*if(page==0){
			if(timer==3) master.print(0, 0, "X:%.1f,Y:%.f,Z:%.1f", accel.x, accel.y, accel.z);
			if(timer==6) master.print(1,0,"Rotation:%f",INERTIA_SENSOR.get_rotation());
			if(abs(INERTIA_SENSOR.get_rotation()) > 360) INERTIA_SENSOR.tare_rotation();
			if(timer==9){	
				master.print(2,0,"Yaw:%f",INERTIA_SENSOR.get_yaw());
				timer=0;
			}
		}
		else if(page==1){
			if(timer==3){
				pros::delay(50);
				master.print(0, 0, "%s", blueRock?"BLUE":"NOT BLUE");
			}
			if(timer==6){
				pros::delay(50);
				master.print(1, 0, "%s", redRock?"RED":"NOT RED");
			}
			if(timer==9){	
				pros::delay(50);
				master.print(2, 0, "%s", noneColor?"NO COLOR": "GOT COLOR");
				timer=0;
			}
		}
		else if(page==2){
			if(timer==3) master.print(0, 0, "Hue:%lf", optical_sensor.get_hue());
			if(timer==6) master.print(1,0,"%d", optical_sensor.get_proximity());
			if(timer==9){
				pros::delay(50);
				master.print(2, 0, "LED Bright: %d%%", ledPWM);
				pros::delay(50);
				//master.print(2, 0, "LED Toggle: %s  ", toggle_led ? "ON" : "OFF");
				timer=0;
			}
		}
		else if(page==3){
			if(timer==3) master.print(0, 0, "Turn kp:%lf", TURN_KD);
			if(timer==6) master.print(1,0,"For kp:%lf", FORWARD_KD);
			if(timer==9){
				timer=0;
			}
		}*/
		if(master.get_digital_new_press(DIGITAL_UP)){
			//auton_fb_target = 20;
			//auton_turn_target = 90;
			autonomous();
		}
		/*if (master.get_digital_new_press(DIGITAL_LEFT)){
			left = true;
			right = false;
		}
		if (master.get_digital_new_press(DIGITAL_RIGHT)) {
			left = false;
			right = true;
		}
		if (master.get_digital_new_press(DIGITAL_UP)){
			left = false;
			right = false;
		}*/
		if (master.get_digital_new_press(DIGITAL_DOWN)){
			SORTING_MOTOR.tare_position();
			SORTING_MOTOR.set_zero_position(0);
			pros::delay(10);
		}
		BTM_ROLLER_MOTOR.move((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)) * powerbtm);
		TOP_ROLLER_MOTOR.move((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2)) * powertop);

		/*
		if(master.get_digital_new_press(DIGITAL_L2)){
			TURN_KD-=0.001;
		}
		if(master.get_digital_new_press(DIGITAL_X)){
			FORWARD_KD+=0.001;
			if(FORWARD_KD>=0.02){
				FORWARD_KD=0.0;
			}
		}*/
		if(optical_sensor.get_proximity() > 220){
			if(hue_value > 210){
				blueRock = true;
				redRock = false;
				noneColor = false;
			}
			else if(hue_value < 6){
				redRock = true;
				blueRock = false;
				noneColor = false;
			}
			else{
				redRock = false;
				blueRock = false;
				noneColor = true;
			}
		}
		else{
			redRock = false;
			blueRock = false;
			noneColor = true;
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
		//followObject();
		pros::delay(10);
		timer++;
	}
}