/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Sachit                                                    */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  2-tile                                                    */
/*    This is 195B's program for the 2022-2023 Vex season and it is very cool */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RearEncoder          encoder       C, D            
// LeftEncoder          encoder       G, H            
// RightEncoder         encoder       E, F            
// Drivetrain           drivetrain    2, 1, 10, 9, 16 
// flywheel             motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
#include <algorithm>
#include <chrono>

using namespace vex;
using code = vision::code;

controller Controller;
competition Competition;
brain brain;

motor intake = motor(PORT8, ratio18_1,false);
motor indexer = motor(PORT5, ratio18_1,false); 

const double KV = 0.075;
const double KP = 0.22;

float target = 128.0;

void autonomous(void){   //Automatic driving
  flywheel.spin(directionType::fwd, 9.1, voltageUnits::volt);
  Drivetrain.driveFor(directionType::fwd, 24, distanceUnits::in);
  Drivetrain.turnToHeading(90, rotationUnits::deg);
  Drivetrain.driveFor(directionType::fwd, 6, distanceUnits::in);
  intake.spinFor(directionType::fwd, 270, rotationUnits::deg);
  Drivetrain.driveFor(directionType::rev, 6, distanceUnits::in);
  Drivetrain.turnToHeading(102.5, rotationUnits::deg);
  indexer.spinFor(directionType::rev, 1.0, timeUnits::sec, 60, velocityUnits::pct);
  vex::task::sleep(1500);
  indexer.spinFor(directionType::rev, 1.50, timeUnits::sec, 60, velocityUnits::pct);
  flywheel.stop(); 
}

void userController(void) {
  while (1) {
  
  rightMotorA.spin(vex::directionType::fwd, Controller.Axis3.value() - Controller.Axis1.value(), vex::velocityUnits::pct);
  rightMotorB.spin(vex::directionType::fwd, Controller.Axis3.value() - Controller.Axis1.value(), vex::velocityUnits::pct);
  leftMotorA.spin(vex::directionType::fwd, Controller.Axis3.value() + Controller.Axis1.value(), vex::velocityUnits::pct);
  leftMotorB.spin(vex::directionType::fwd, Controller.Axis3.value() + Controller.Axis1.value(), vex::velocityUnits::pct);
 
  if( Controller.ButtonR1.pressing()){
    intake.spin(directionType::rev, 100, velocityUnits::pct);
  }
  else if( Controller.ButtonR2.pressing()){
    intake.spin(directionType::fwd, 100, velocityUnits::pct);
  }
  else {
    intake.stop(brakeType::brake);
  }
  if( Controller.ButtonL2.pressing()){
    float error = target - flywheel.velocity(velocityUnits::rpm);
    flywheel.spin(directionType::fwd, std::min(target * KV + error * KP, 12.0), voltageUnits::volt);
  }
  else {
    flywheel.stop(brakeType::brake);
  }
  if ( Controller.ButtonL1.pressing()){
    indexer.spin(directionType::fwd, 75, velocityUnits::pct);
  }
  else {
    indexer.stop(brakeType::brake);
  }

  if( Controller.ButtonUp.pressing()){
    target=160;
    wait(10, msec);
  }
  else if(Controller.ButtonDown.pressing()) {
    target=96;
    wait(10, msec);
  }
  else if(Controller.ButtonLeft.pressing()) {
    target=112;
    wait(10, msec);
  }
  else if(Controller.ButtonRight.pressing()) {
    target=128;
    wait(10, msec);
  }

  vex::task::sleep(100);
}
}

int main() {  
  vexcodeInit();
  flywheel.setMaxTorque(1.5, currentUnits::amp);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userController);

  int iteration = 0;
  while (true){
    if (iteration % 10)
    {
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(0, 0);
      Controller.Screen.print("Target RPM:");
      Controller.Screen.newLine();
      Controller.Screen.print("Flywheel RPM:", (int)flywheel.velocity(velocityUnits::rpm));
      Controller.Screen.newLine();
      Controller.Screen.setCursor(1, 13);
      Controller.Screen.print("        ");
      Controller.Screen.setCursor(1, 13);
      Controller.Screen.print("%d", (int) target);
      Controller.Screen.setCursor(2, 19);
      Controller.Screen.print("        ");
      Controller.Screen.setCursor(2, 19);
      Controller.Screen.print("%d", (int)flywheel.velocity(velocityUnits::rpm));
      Controller.Screen.setCursor(3, 2);
      Controller.Screen.print("        ");
      Controller.Screen.setCursor(3, 2);
      Controller.Screen.print("%d", Brain.Battery.capacity(percentUnits::pct));
      Controller.Screen.setCursor(3, 5);
      Controller.Screen.print("        ");
      Controller.Screen.setCursor(3, 5);
      Controller.Screen.print("%");
    }
    iteration++;
    wait(100, timeUnits::msec);
  }
}