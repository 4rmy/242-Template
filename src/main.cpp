#include "main.h"         
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "firebird_template/lcd/lcd.h"

#include "autons.hpp"


void initialize() {
    lcd::initialize(); // overwrite pros LCD
    chassis.calibrate(); // calibrate sensors
    auton_init(); // run auton selector
}

void disabled() { /* kill the auto selector here if possible */ }

void competition_initialize() { /* don't use this bc it's bad practice */ }

/* Auton Handling */
void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // hold brakes for more accurate autos
    as::call_selected_auton(); // run the selected auton
}

void opcontrol() {
    while (true) {
        // delay to keep tasks running
        pros::delay(10);
    }
}