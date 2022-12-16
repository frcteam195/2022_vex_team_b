#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
encoder RearEncoder = encoder(Brain.ThreeWirePort.C);
encoder LeftEncoder = encoder(Brain.ThreeWirePort.G);
encoder RightEncoder = encoder(Brain.ThreeWirePort.E);
motor leftMotorA = motor(PORT2, ratio18_1, true);
motor leftMotorB = motor(PORT1, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT10, ratio18_1, false);
motor rightMotorB = motor(PORT9, ratio18_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT16);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 12.9591, 14.5, 12.0, distanceUnits::in, 1);
motor flywheelMotorA = motor(PORT18, ratio18_1, true);
motor flywheelMotorB = motor(PORT20, ratio18_1, false);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);
// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}