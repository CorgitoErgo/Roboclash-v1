#include "main.h"

bool sorting_enable = true;
bool blue_alliance = true;
int Top_Roller_State = 3;
int Btm_Roller_State = 3;

/*Check definitions header file for port assignments*/
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor LEFT_MOTOR_FRONT(LEFT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_FRONT(RIGHT_MOTOR_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RIGHT_MOTOR_REAR(RIGHT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LEFT_MOTOR_REAR(LEFT_MOTOR_REAR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor SORTING_MOTOR(SORTING_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BTM_ROLLER_MOTOR(BOTTOM_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor TOP_ROLLER_MOTOR(TOP_ROLLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor EVAC_MOTOR(EVAC_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Optical OPTICAL_SENSOR(OPTICAL_PORT, OPTICAL_PORT_UPDATE);
pros::c::optical_rgb_s_t rgb_value;

double hue_value = 0;
int proximity = 0;
bool blueRock = false;
bool redRock = false;
bool noColor = true;

void sortingTask() {
	OPTICAL_SENSOR.set_integration_time(40);
	OPTICAL_SENSOR.set_led_pwm(50);

	SORTING_MOTOR.set_zero_position(0);
	SORTING_MOTOR.tare_position();

	while (true) {
		if(sorting_enable==true){
			if(OPTICAL_SENSOR.get_led_pwm() == 0) OPTICAL_SENSOR.set_led_pwm(50);
			hue_value = OPTICAL_SENSOR.get_hue();
			proximity = OPTICAL_SENSOR.get_proximity();
			if(hue_value > 220 && hue_value < 245 && proximity >= 230){
				blueRock = true;
				redRock = false;
				noColor = false;
			}
			else if(hue_value < 11 && proximity >= 230){
				redRock = true;
				blueRock = false;
				noColor = false;
			}
			else{
				redRock = false;
				blueRock = false;
				noColor = true;
			}
			
			if(blue_alliance == true){
				if(blueRock == true){
					blueRock = false;
					SORTING_MOTOR.move_absolute(-50, 95);
					pros::delay(600);
					SORTING_MOTOR.move_absolute(0, 130);
					pros::delay(200);
				}
				else if(redRock == true){
					redRock = false;
					SORTING_MOTOR.move_absolute(50, 95);
					pros::delay(600);
					SORTING_MOTOR.move_absolute(0, 130);
					pros::delay(200);
				}
				else if(noColor == true){
					SORTING_MOTOR.move_absolute(0, 130);
					pros::delay(100);
				}
				else{
					SORTING_MOTOR.move_absolute(0, 130);
				}
			}
			else{
				if(blueRock == true){
					blueRock = false;
					SORTING_MOTOR.move_absolute(50, 95);
					pros::delay(600);
					SORTING_MOTOR.move_absolute(0, 130);
					pros::delay(200);
				}
				else if(redRock == true){
					redRock = false;
					SORTING_MOTOR.move_absolute(-50, 95);
					pros::delay(600);
					SORTING_MOTOR.move_absolute(0, 130);
					pros::delay(200);
				}
				else if(noColor == true){
					SORTING_MOTOR.move_absolute(0, 130);
					pros::delay(100);
				}
				else{
					SORTING_MOTOR.move_absolute(0, 130);
				}
			}
			pros::delay(10);
		}
		else{
			if(OPTICAL_SENSOR.get_led_pwm() == 50) OPTICAL_SENSOR.set_led_pwm(0);
			pros::delay(2);
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
	pros::lcd::initialize();
	pros::lcd::set_text(0, "[Suckers gonna Suck]");
	pros::lcd::set_text(1, "1. John Chai");
	pros::lcd::set_text(2, "2. Nawawi");
	pros::lcd::set_text(3, "3. Cheng Yi");
	pros::lcd::set_text(4, "4. Hans");
	pros::lcd::set_background_color(0,128,128);
	pros::lcd::set_text_color(LV_COLOR_BLACK);

	OPTICAL_SENSOR.disable_gesture();

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

void autonomous() {}

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

void full_brake(){
	LEFT_MOTOR_FRONT.brake();
	RIGHT_MOTOR_FRONT.brake();
	LEFT_MOTOR_REAR.brake();
	RIGHT_MOTOR_REAR.brake();
	pros::delay(1);
}

/*void print_screen(){
	master.clear();
	pros::delay(30);
	master.print(0,0,"%s", "X-sort,Y-alliance");
	pros::delay(30);
	master.print(1,0,"Sort-%s, B-Brake", sorting_enable ? "On":"Off");
	pros::delay(30);
	master.print(2,0,"Alliance-%s", blue_alliance ? "Blue":"Red");
}*/

void opcontrol() {
	master.clear();
	pros::delay(50);
	master.print(0,0,"%s","Y:Alliance,X:Sorter");
	pros::delay(50);
	master.print(1,0,"%s","B:Brake,A:TareSort");
	pros::delay(50);
	master.print(2,0,"[Sort: %s]", sorting_enable ? "On ":"Off");
	pros::delay(50);
	master.print(2,10,"[Ally: %s]",blue_alliance ? "Blue":"Red ");
	EVAC_MOTOR.tare_position();

	while (true) {
		Top_Roller_State = (master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2));
		
		Btm_Roller_State = (master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2));
		
		//EVAC_MOTOR.move_absolute((master.get_digital(DIGITAL_UP) - master.get_digital(DIGITAL_DOWN)) * powerevac);
		if(master.get_digital_new_press(DIGITAL_UP)) EVAC_MOTOR.move_absolute(1500, 130);

		if(master.get_digital_new_press(DIGITAL_DOWN)) EVAC_MOTOR.move_absolute(0, 125);

		if(master.get_digital_new_press(DIGITAL_X)){
			sorting_enable = !sorting_enable;
			if(sorting_enable) OPTICAL_SENSOR.set_led_pwm(50);
			else OPTICAL_SENSOR.set_led_pwm(0);
			master.print(2,0,"[Sort: %s]", sorting_enable ? "On ":"Off");
			pros::delay(2);
		}

		if(master.get_digital_new_press(DIGITAL_Y)){
			blue_alliance = !blue_alliance;
			master.print(2,10,"[Ally: %s]",blue_alliance ? "Blue":"Red ");
		}

		if(master.get_digital_new_press(DIGITAL_A)) SORTING_MOTOR.tare_position();

		while(master.get_digital(DIGITAL_B)) full_brake();

		if(sorting_enable == false){
			SORTING_MOTOR.move((master.get_digital(DIGITAL_LEFT) - master.get_digital(DIGITAL_RIGHT))*20);
		}
		/*ARCADE DRIVE*/
		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		LEFT_MOTOR_FRONT.move(power + turn);
		LEFT_MOTOR_REAR.move(power + turn);
		RIGHT_MOTOR_FRONT.move(power - turn);
		RIGHT_MOTOR_REAR.move(power - turn);

		/*TANK DRIVE*/
		/*
		int powerLeft = master.get_analog(ANALOG_LEFT_Y);
		int powerRight = master.get_analog(ANALOG_RIGHT_Y);
		LEFT_MOTOR_FRONT = powerLeft;
		LEFT_MOTOR_REAR = powerLeft;
		RIGHT_MOTOR_FRONT = powerRight;
		RIGHT_MOTOR_REAR = powerRight;
		*/
		pros::delay(5);
		//print_screen();
	}
}
