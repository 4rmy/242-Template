#include "main.h" // IWYU pragma: keep
	#include "lemlib/api.hpp" // IWYU pragma: keep
	#include "lemlib/chassis/chassis.hpp"
	#include "lemlib/chassis/trackingWheel.hpp"
	#include "lemlib/pid.hpp" // IWYU pragma: keep
	#include "lemlib/pose.hpp"
	#include "liblvgl/core/lv_disp.h" // IWYU pragma: keep
	#include "liblvgl/core/lv_obj.h" // IWYU pragma: keep
	#include "liblvgl/core/lv_obj_pos.h" // IWYU pragma: keep
	#include "liblvgl/core/lv_obj_style.h" // IWYU pragma: keep
	#include "liblvgl/core/lv_obj_tree.h" // IWYU pragma: keep
	#include "liblvgl/misc/lv_color.h" // IWYU pragma: keep
	#include "liblvgl/misc/lv_style.h" // IWYU pragma: keep
	#include "liblvgl/widgets/lv_img.h" // IWYU pragma: keep
	#include "pros/abstract_motor.hpp" // IWYU pragma: keep
	#include "pros/adi.hpp" // IWYU pragma: keep
	#include "pros/colors.hpp" // IWYU pragma: keep
	#include "pros/distance.hpp" // IWYU pragma: keep
	#include "pros/llemu.hpp" // IWYU pragma: keep
	#include "pros/misc.h" // IWYU pragma: keep
	#include "pros/misc.hpp"
	#include "pros/motor_group.hpp" // IWYU pragma: keep
	#include "pros/motors.h"
	#include "pros/motors.hpp" // IWYU pragma: keep
	#include <cstdio> // IWYU pragma: keep
	#include <cwchar> // IWYU pragma: keep
	#include <string> // IWYU pragma: keep
	#include "as.h" // IWYU pragma: keep
	#include "pros/optical.hpp" // IWYU pragma: keep
	#include "pros/rotation.hpp" // IWYU pragma: keep
	#include "pros/rtos.hpp"
	#include "pros/vision.hpp" // IWYU pragma: keep


// LemLib Setup
pros::MotorGroup left_motors({0,0,0}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({0,0,0}, pros::MotorGearset::blue);
pros::Imu imu(3);

/*
	pros::Rotation vert(4);
	lemlib::TrackingWheel vert_tracking(&vert, 2.125, 1.875, 1.0);
	pros::Rotation horiz(5);
	lemlib::TrackingWheel horiz_tracking(&horiz, 2.125, 0.375, 1.0);
*/

float track_width = 0;
float wheel_diameter = lemlib::Omniwheel::NEW_325;
float rpm = 450.0;
float horizontal_drift = 8.0;

// drivetrain controller
lemlib::Drivetrain drivetrain(
	&left_motors,
	&right_motors,
	track_width,
	wheel_diameter,
	rpm,
	horizontal_drift
);

// sensors controller
lemlib::OdomSensors sensors(
	nullptr, // vertical tracking wheel 1, set to null
	nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
	nullptr, // horizontal tracking wheel 1
	nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
	&imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	15, // proportional gain (kP)
	0, // integral gain (kI)
	20, // derivative gain (kD)
	0, // anti windup
	0.1, // small error range, in inches
	100, // small error range timeout, in milliseconds
	1, // large error range, in inches
	500, // large error range timeout, in milliseconds
	20 // maximum acceleration (slew)
);

// angular PID controllerk
lemlib::ControllerSettings angular_controller(
	15, // proportional gain (kP)
	0, // integral gain (kI)
	30, // derivative gain (kD)
	0, // anti windup
	0.5, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	1, // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

// chassis controller
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
	lateral_controller, // lateral PID settings
	angular_controller, // angular PID settings
	sensors // odometry sensors
);

// Devices
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalIn auton_btn('f');

void auton_init();

void initialize() {
	lcd::initialize();
    chassis.calibrate(); // calibrate sensors
	auton_init();
	pros::Task as_button([&]() {
		bool new_press = true;
		
		while (true) {
			if (pros::competition::is_disabled()) {
				if (auton_btn.get_value() == 1) {
					if (new_press) {
						as::increment();
						new_press = false;
					}
				} else {
					new_press = true;
				}
			} else {
				break;
			}
			pros::delay(20);
		}
	});
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	lcd::clear();
	pros::Task screen_task([&]() {
	  	while (true) {
            // print robot location to the brain screen
            lcd::print(0, "X: " + std::to_string(chassis.getPose().x)); // x
            lcd::print(1, "Y: " + std::to_string(chassis.getPose().y)); // y
            lcd::print(2, "Theta: " + std::to_string(chassis.getPose().theta)); // heading

			// delay to save resources
            pros::delay(20);
        }
    });
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setPose(lemlib::Pose(0, 0, 0), false);
	as::call_selected_auton();
}

void opcontrol() {
	lcd::clear();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	while (true) {
		// get left y and right x positions
		int Y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int X = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		// move the robot
		chassis.arcade(Y, -X);

		// delay to keep tasks running
		pros::delay(10);
	}
}

/*

	AUTON FUNCTIONS

*/
void auton_1() {
	// auton code here
}

void auton_init() {
	as::add_auton(as::Auton("Auton 1", "Simple starter auton", auton_1));

	as::init();
}
