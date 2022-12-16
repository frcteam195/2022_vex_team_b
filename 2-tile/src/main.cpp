/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Sachit                                                    */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  2-tile                                                    */
/*    This example allows you to control the V5 Clawbot using the left        */
/*    joystick. Adjust the deadband value for more accurate movements.        */
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

// motor rightmotorfront = motor(PORT2, ratio18_1,false);
// motor rightmotorrear = motor(PORT1, ratio18_1,false);
// motor leftmotorfront = motor(PORT9, ratio18_1,false);
// motor leftmotorrear = motor(PORT10, ratio18_1,false);
motor intake = motor(PORT8, ratio18_1,false);
motor indexer = motor(PORT5, ratio18_1,false); 

//drivetrain LeftTrain = drivetrain(LeftFrontMotor, LeftRearMotor);
//drivetrain RightTrain = drivetrain(RightFrontMotor, RightRearMotor);
//drivetrain TailTrain = drivetrain(RightTail, RightTail);

/*int a = 1;
void toggleP1(){
  if (a == 1) {
    B1.spin(forward);
    R1.spin(forward);
    R2.spin(forward);
    B2.spin(forward);
   a = 0;
  }
  else {
    B1.stop();
    R1.stop();
    B2.stop();
    R2.stop();
   a = 1;
  }
}
void toggleR1(){
  if (a == 1) {
    R1.spin(forward);
    R2.spin(forward);
   a = 0;
  }
  else {
    B1.stop();
    R1.stop();
    B2.stop();
    R2.stop();
   a = 1;
  }
}

void toggleB1(){
  if (a == 1) {
    B1.spin(forward);
    B2.spin(forward);
   a = 0;
  }
  else {
    B1.stop();
    R1.stop();
    B2.stop();
    R2.stop();
   a = 1;
  }
}*/

const double KV = 0.075;
const double KP = 0.22;

float target = 128.0;

// void decreaseTarget(void)
// {
//   target =  fmax(target - 10.0, 400.0);
// }

// void increaseTarget(void)
// {
//   target = fmin(target + 10.0, 600.0);
// }

void autonomous(void){   //Automatic driving
  /*flywheel.spin(directionType::fwd, 9.25, voltageUnits::volt);
  Drivetrain.driveFor(directionType::rev, 6 , distanceUnits::in);
  Drivetrain.turnToHeading(25, rotationUnits::deg);
  vex::task::sleep(300);
  indexer.spinFor(directionType::rev, 1.0, timeUnits::sec, 60, velocityUnits::pct);
  vex::task::sleep(1100);
  indexer.spinFor(directionType::rev, 1.50, timeUnits::sec, 60, velocityUnits::pct);
  flywheel.stop();
  Drivetrain.turnToHeading(270, rotationUnits::deg, 40, velocityUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 25, distanceUnits::in, 60, velocityUnits::pct);
  Drivetrain.turnToHeading(355, rotationUnits::deg, 40, velocityUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 9, distanceUnits::in, 60, velocityUnits::pct);
  intake.spinFor(directionType::fwd, 270, rotationUnits::deg);
  Drivetrain.driveFor(directionType::rev, 8, distanceUnits::in, 100, velocityUnits::pct);*/
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
  
  // float error = target - flywheel.velocity(velocityUnits::rpm);
  // flywheel.spin(directionType::fwd, std::min(target * KV + error * KP, 12.0), voltageUnits::volt);
  // flywheel.spin(directionType::rev, 70, velocityUnits::pct);
  // Drivetrain.driveFor(forward,);

}


void userController(void) {
 /*R1.setVelocity(50, pct);
 B1.setVelocity(50, pct);*/
  while (1) {

//  int x = Controller.Axis4.value();
//  int y = Controller.Axis3.value();
 //int y1 = Controller.Axis1.value();

 



//  int xy = (y + x);
//  int yx = (y - x);

//  leftmotorfront.spin(directionType::fwd, (yx), percentUnits::pct);
//  leftmotorrear.spin(directionType::fwd, (yx), percentUnits::pct);
//  rightmotorfront.spin(directionType::rev, (xy), percentUnits::pct);
//  rightmotorrear.spin(directionType::rev, (xy), percentUnits::pct);

  //std::cout << "Axis 3: " << Controller.Axis3.value() << std::endl;
 
  rightMotorA.spin(vex::directionType::fwd, Controller.Axis3.value() - Controller.Axis1.value(), vex::velocityUnits::pct);
  rightMotorB.spin(vex::directionType::fwd, Controller.Axis3.value() - Controller.Axis1.value(), vex::velocityUnits::pct);
  leftMotorA.spin(vex::directionType::fwd, Controller.Axis3.value() + Controller.Axis1.value(), vex::velocityUnits::pct);
  leftMotorB.spin(vex::directionType::fwd, Controller.Axis3.value() + Controller.Axis1.value(), vex::velocityUnits::pct);
  //Drivetrain.arcade(double drivePower, double turnPower)

  


 //std::cout << "Left Encoder: " << LeftEncoder.value()  << std::endl;
 //std::cout << "Rear Encoder: " << RearEncoder.value()  << std::endl;
 //std::cout << "Right Encoder: " << RightEncoder.value()*2  << std::endl;
 //std::cout << "Average Left + Right: " << ((LeftEncoder.value() + (RightEncoder.value()*2))/2) << std::endl;

 //std::cout << "Heading: " << DrivetrainInertial.heading(degrees)  << std::endl;

 /*R1.spin(directionType::fwd, (yx), percentUnits::pct);
 B1.spin(directionType::fwd, (yx), percentUnits::pct);
 R2.spin(directionType::fwd, (yx), percentUnits::pct);
 B2.spin(directionType::fwd, (yx), percentUnits::pct);
 Ground1.spin(directionType::fwd, (yx), percentUnits::pct);
 Ground2.spin(directionType::fwd, (yx), percentUnits::pct);*/
 
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
  if /*(speed-2<(flywheel.velocity(velocityUnits::pct))&&(flywheel.velocity(velocityUnits::pct))<speed+2 &&*/ (Controller.ButtonL1.pressing()){
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