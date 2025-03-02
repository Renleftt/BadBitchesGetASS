#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
pros::Controller master(pros::E_CONTROLLER_MASTER);
// Be careful with this inclusion!

/**
 * A callback function for LLEMU's center button.
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
 * Runs initialization code.
 */
 pros::Motor R1(1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 pros::Motor R2(2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 pros::Motor R3(3, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 pros::Motor R4(4, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 
 pros::Motor L1(5, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 pros::Motor L2(6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 pros::Motor L3(7, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 pros::Motor L4(8, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 
 // Defining the Arm motors
 pros::Motor A1(16, pros::v5::MotorGears::green, pros::v5::MotorUnits::counts);
 pros::Motor A2(17, pros::v5::MotorGears::green, pros::v5::MotorUnits::counts);
 
 // Defining the Intake Motors 
 pros::Motor I1(18, pros::v5::MotorGears::green, pros::v5::MotorUnits::counts);
 pros::Motor I2(19, pros::v5::MotorGears::green, pros::v5::MotorUnits::counts);
 pros::IMU imu_sensor(11);
 pros::MotorGroup leftMotors({1, -2, 3, -4});
 pros::MotorGroup rightMotors({5, -6, 7,-8}); 
 pros::MotorGroup intakeMotors({9, -10,}); 
 pros::MotorGroup armGroup({11,17});

pros::adi::DigitalOut solenoidE('E'); // Hold-based solenoid
pros::adi::DigitalOut solenoidF('F'); // Starts extended (toggle)
pros::adi::DigitalOut solenoidG('G'); // Starts extended (toggle)
pros::adi::DigitalOut solenoidH('H');

void initialize(){
    imu_sensor.reset();  // Reset and calibrate the IMU
    pros::lcd::print(0, "Calibrating IMU...");
    pros::delay(2500);   // Allow time for calibration
    pros::lcd::print(0, "IMU Ready!");
}

/**
 * Runs while the robot is in the disabled state.
 */
void disabled() {}

/**
 * Runs before autonomous when connected to the Field Management System.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
 void moveForward(double distance, int speed) {
    double target = distance * 360 / (4 * M_PI); // Convert inches to encoder counts
    leftMotors.tare_position();
    rightMotors.tare_position();

    while (fabs(leftMotors.get_position()) < fabs(target)) {
        leftMotors.move_velocity(speed);
        rightMotors.move_velocity(speed);
        pros::delay(20);
    }

    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

// Turn to a precise angle using IMU
void turnToAngle(double targetAngle) {
    double error = targetAngle - imu_sensor.get_heading();

    while (fabs(error) > 2) { // Stops when error is within 2 degrees
        double speed = error * 1.2; // P-control for smooth turn
        leftMotors.move_velocity(-speed);
        rightMotors.move_velocity(speed);

        pros::delay(20);
        error = targetAngle - imu_sensor.get_heading();
    }

    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

// Full autonomous routine
void autonomous() {
    pros::lcd::print(0, "Autonomous Started!");
    
    moveForward(24, 100);
    pros::delay(500);
    
    turnToAngle(90);
    pros::delay(500);
    
    moveForward(12, 80);
    pros::delay(500);
    
    turnToAngle(180);
    pros::delay(500);
    
    moveForward(-24, 100);
    
    pros::lcd::print(0, "Autonomous Complete!");
}
/**
 * Runs the operator control code.
 */

 
//Defining the motor groups
void setMotorCoast() {
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    armGroup.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

//Speed for motors is 100 for now
void driveForward (int speed) {
    leftMotors.move_velocity(speed);
    rightMotors.move_velocity(speed);;
}

void driveBackward(int speed) {
    leftMotors.move_velocity(-speed);  // Negative speed for backward motion
    rightMotors.move_velocity(-speed);
}

void turnLeft(int speed) {
    leftMotors.move_velocity(-speed);  // Negative speed for left motors to turn left
    rightMotors.move_velocity(speed);  // Positive speed for right motors
}

void turnRight(int speed) {
    leftMotors.move_velocity(speed);   // Positive speed for left motors
    rightMotors.move_velocity(-speed); // Negative speed for right motors to turn right
}

void intake(int speed){
    intakeMotors.move_velocity(speed);
}
void toggleSolenoidE() {
    static bool eState = true;
    eState = !eState;
    solenoidE.set_value(eState);
}

void toggleSolenoidF() {
    static bool fState = true;
    fState = !fState;
    solenoidF.set_value(fState);
}

void toggleSolenoidG() {
    static bool gState = true;
    gState = !gState;
    solenoidG.set_value(gState);
}

void toggleSolenoidH() {
    static bool hState = true;
    hState = !hState;
    solenoidH.set_value(hState);
}

bool motorsActivated = false;

void pistons(){
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        toggleSolenoidF();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        toggleSolenoidE();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        toggleSolenoidG();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        toggleSolenoidH();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        motorsActivated = !motorsActivated;
        if (motorsActivated) {
            intakeMotors.move_velocity(100); // Run intake
        } else {
            intakeMotors.move_velocity(0); // Stop intake
        }
    }   
}
void opcontrol() {
    setMotorCoast();
    while (true) {
     // Set motors to coast mode
        pistons();
        pros::delay(60);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            driveForward(127);  // Drive forward at full speed
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            driveBackward(127); // Drive backward at full speed
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            turnLeft(127);      // Turn left at full speed
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            turnRight(127);     // Turn right at full speed
        } else {
            // Stop all motors if no controls are active
            leftMotors.move_velocity(0);
            rightMotors.move_velocity(0);
        }

        // Get joystick values for tank drive control   
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);  // Forward/backward movement
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // Turning movement

        // Apply movement to motors
        leftMotors.move(dir + turn);  // Left motor control
        rightMotors.move(dir - turn); // Right motor control

        pros::delay(100); // Prevent CPU overload by adding a small delay
    }
}
