#include "main.h"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
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
 
 void initialize(){
    pros::Motor R1(1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 	pros::Motor R2(2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 	pros::Motor R3(3, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 	pros::Motor R4(4, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
 
 	pros::Motor L1(5, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	pros::Motor L2(6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	pros::Motor L3(7, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	pros::Motor L4(8, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	
	// Defining the Arm motors
	pros::Motor A1(16, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	pros::Motor A2(17, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	
	// Defining the Intake Motors 
	pros::Motor I1(18, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	pros::Motor I2(19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
	// Initialize the solenoid control on port 'A'
	pros::adi::DigitalOut solenoid('A');

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
void autonomous() {}

/**
 * Runs the operator control code.
 */


 
//Defining the motor groups
pros::MotorGroup leftMotors({1, -2, 3, -4, 5});
pros::MotorGroup rightMotors({-13, 20,-8,9,-18}); 

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

void opcontrol() {
    while (true) {
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

        pros::delay(20); // Prevent CPU overload by adding a small delay
    }
}





pros::adi::DigitalOut solenoidA('A'); // Solenoid on port 'A'
pros::adi::DigitalOut solenoidB('B'); // Solenoid on port 'B'

bool pneumaticWork = false; // Tracks the state of the piston

void extendPiston() {
    solenoidA.set_value(true); // Extend the piston
    solenoidB.set_value(true); // Extend the piston
}

void retractPiston() {
    solenoidA.set_value(false); // Retract the piston
    solenoidB.set_value(false); // Retract the piston
}

void controlPiston() {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            extendPiston(); // Extend the piston with L1 
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            retractPiston(); // Retract the piston with L2
        }
        pros::delay(50); // Small delay for button press processing
    }
}

int main() {
    pros::Task pistonControl(controlPiston); // Runs the piston control

    while (true) {
        pros::delay(100); // Delay 
    }
}
