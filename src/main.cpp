#include "main.h"
#include "lemlib/api.hpp"

// ..................................................................................
// ..................................................................................

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor FL(11, pros::E_MOTOR_GEARSET_06, false); // port 11, blue gearbox, not reversed
pros::Motor BL(12, pros::E_MOTOR_GEARSET_06, false); // port 12, blue gearbox, not reversed
pros::Motor TL(1, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed
pros::Motor FR(9, pros::E_MOTOR_GEARSET_06, true); // port 9, blue gearbox, reversed
pros::Motor BR(8, pros::E_MOTOR_GEARSET_06, true); // port 8, blue gearbox, reversed
pros::Motor TR(10, pros::E_MOTOR_GEARSET_06, true); // port 10, blue gearbox, reversed
pros::Motor Cata(16, pros::E_MOTOR_GEARSET_36, false); // port 16, red gearbox, not reveresed
pros::Motor Intake(9, pros::E_MOTOR_GEARSET_06, false); // port 16, red gearbox, not reveresed
pros::MotorGroup Leftsidedrive({FL, BL, TL});
pros::MotorGroup Rightsidedrive({FR, BR, TR});
lemlib::Drivetrain_t drivetrain {
	&Leftsidedrive, // left drivetrain motors
	&Rightsidedrive, // right drivetrain motors
	11.3, // track width
	3.25, // wheel diameter
	360 // wheel rpm
};

pros::Rotation rot(3, false); // port 3, not reversed
pros::Rotation brot(4, false); // port 4, not reversed
lemlib::TrackingWheel left_tracking_wheel(&rot, 2.75, 4.3, 2); // uses "rot" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 2:1 gear ratio
lemlib::TrackingWheel back_tracking_wheel(&brot, 2.75, 4.3, 2); // uses "brot" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 2:1 gear ratio

// left tracking wheel encoder
pros::Rotation left_rot(4, false); // port 4, not reversed

// back tracking wheel encoder
pros::Rotation back_rot(5, false); // port 5, not reversed

// left tracking wheel
lemlib::TrackingWheel left_tracking_wheel(&left_rot, 2.75, -4.6); // 2.75" wheel diameter, -4.6" offset from tracking center
// right tracking wheel
lemlib::TrackingWheel back_tracking_wheel(&back_rot, 2.75, 4.5); // 2.75" wheel diameter, 4.5" offset from tracking center

// inertial sensor
pros::Imu inertial_sensor(16); // port 16
 
// odometry struct
lemlib::OdomSensors_t sensors {
	&left_tracking_wheel, // vertical tracking wheel 1
	nullptr, // vertical tracking wheel 2
	&back_tracking_wheel, // horizontal tracking wheel 1
	nullptr, // we don't have a second tracking wheel, so we set it to nullptr
	&inertial_sensor // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
	8, // kP
	30, // kD
	1, // smallErrorRange
	100, // smallErrorTimeout
	3, // largeErrorRange
	500, // largeErrorTimeout
	5 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController {
	4, // kP
	40, // kD
	1, // smallErrorRange
	100, // smallErrorTimeout
	3, // largeErrorRange
	500, // largeErrorTimeout
	0 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// ..................................................................................
// ..................................................................................

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

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
	// timeout: 2000 ms
	// lookahead distance: 15 inches
	chassis.follow("offside match1.txt", 2000, 15);
	Intake.move_velocity(600);
	chassis.follow("offside match2.txt", 2000, 15);
	Intake.brake();
	chassis.follow("offside match3.txt", 2000, 15);
	chassis.follow("offside match4.txt", 2000, 15);
	chassis.follow("offside match5.txt", 2000, 15);
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

	while(true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		
		// Retrieve the necessary joystick values
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		// Move the left side of the robot
		Leftsidedrive.move(leftY + rightX);
		
		// Move the right side of the robot 
		Rightsidedrive.move(leftY - rightX);
		pros::delay(20);
	}
}
