#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

inline auto CCW_COUNTERCLOCKWISE = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
inline auto CW_CLOCKWISE = lemlib::AngularDirection::CW_CLOCKWISE;

inline pros::MotorGroup left_motors({10, -8, -9}, pros::MotorGearset::blue);
inline pros::MotorGroup right_motors({-19, 6, 5}, pros::MotorGearset::blue);
inline pros::MotorGroup intake({20, -4}, pros::v5::MotorGears::green);
inline pros::Motor lady_brown(-18, pros::v5::MotorGears::green);
inline pros::Imu imu(15);
inline pros::adi::Pneumatics clamp('a', false);
inline pros::adi::Pneumatics doinker('b', false);
inline pros::adi::Pneumatics rej_piston('c', false);
inline pros::Optical color(14);
inline pros::adi::Pneumatics intake_lift('h', false);

inline pros::Rotation vert(-17);
inline lemlib::TrackingWheel vert_tracking(&vert, 2, 1.125, 1.0);
inline pros::Rotation horiz(16);
inline lemlib::TrackingWheel horiz_tracking(&horiz, 2, -2.75, 1.0);

// constants
inline float track_width = 11;
inline float wheel_diameter = lemlib::Omniwheel::NEW_275;
inline float rpm = 450.0;
inline float horizontal_drift = 2.0;

// drivetrain controller
inline lemlib::Drivetrain
    drivetrain(
        &left_motors,
        &right_motors,
        track_width,
        wheel_diameter,
        rpm, 
        horizontal_drift
    );

// sensors controller
inline lemlib::OdomSensors
    sensors(
        &vert_tracking,
        nullptr,
        &horiz_tracking,
        nullptr,
        &imu
    );

// lateral PID controller
inline lemlib::ControllerSettings
    lateral_controller(
        10,  // proportional gain (kP)
        0,   // integral gain (kI)
        3,   // derivative gain (kD)
        3,   // anti windup
        1,   // small error range, in inches
        100, // small error range timeout, in milliseconds
        3,   // large error range, in inches
        500, // large error range timeout, in milliseconds
        20   // maximum acceleration (slew)
    );

// angular PID controllerk
inline lemlib::ControllerSettings
    angular_controller(
        2,   // proportional gain (kP)
        0,   // integral gain (kI)
        10,  // derivative gain (kD)
        3,   // anti windup
        1,   // small error range, in degrees
        100, // small error range timeout, in milliseconds
        3,   // large error range, in degrees
        500, // large error range timeout, in milliseconds
        0    // maximum acceleration (slew)
    );

// chassis controller
inline lemlib::Chassis
    chassis(
        drivetrain,
    lateral_controller,
    angular_controller,
        sensors
    );

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);
