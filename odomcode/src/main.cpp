/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel (saurinpatel and noahgelbart)                  */
/*    Created:                                                 */
/*    Description:  Odometry For Precise Autonomous Motion                    */
/*    Credits:      5225A For Pilons POS Document and QUEENS for odom Alg.    */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightBack            motor         9               
// RightFront           motor         10              
// LeftFront            motor         1               
// LeftBack             motor         2               
// Controller1          controller                    
// EncoderC             encoder       C, D            
// EncoderE             encoder       E, F            
// EncoderG             encoder       G, H            
// Gyro                 inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
#include <iostream>
using namespace vex;
vex::competition    Competition;
/*-------------------------------Variables-----------------------------------*/
#define Pi 3.14159265358979323846
#define fieldscale 1.66548042705
//#define fieldscale 1.625
//#define fieldscale 1.83
#define SL 3.5 //distance from tracking center to middle of left wheel
#define SR 3.5 //distance from tracking center to middle of right wheel
#define SS 2 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 4.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation

double DeltaL = 0;
double DeltaR = 0;
double DeltaB = 0;

int currentL = 0;
int currentR = 0;
int currentB = 0;

int PreviousL = 0;
int PreviousR = 0;
int PreviousB = 0;

double DeltaTheta = 0;
double DeltaXBack = 0;
double DeltaYBack = 0;

double X = 0;
double Y = 0;
double Theta = 0;

double DeltaXSide = 0;
double DeltaYSide = 0;
double SideChord = 0;
double BackChord = 0;
double OdomHeading = 0;

/*---------------------------------------------------------------------------*/
/*                            Odometry Functions                             */
/*---------------------------------------------------------------------------*/
bool getLeftEncoderDegrees(int &output)
{
  output = EncoderG.position(degrees);
  return (-1000000 < output && output < 1000000);
}

bool getRearEncoderDegrees(int &output)
{
  output = EncoderC.position(degrees);
  return (-1000000 < output && output < 1000000);
}

bool getRightEncoderDegrees(int &output)
{
  output = EncoderE.position(degrees);
  return (-1000000 < output && output < 1000000);
}

void TrackPOS() {
// 2 cases could be occuring in odometry
// 1: Going in a straight line
// 2: Going in an arc motion
// If the bot is on an angle and going straight the displacement would be linear at angle Theta, meaning a right triangle is formed (Trig ratios to calc movement)
// Since it is a linear motion, the Left and right will move the same amount so we can just pick a side and do our movement calculation
// Since this calculation is working based of very infinitely small arcs, the displacement of the robot will be a chord
// Below it Averages the Left and Right integrated motor encoders since we don't have encoders yet

  // if (!getRightEncoderDegrees(currentR) || !getLeftEncoderDegrees(currentL) || !getRearEncoderDegrees(currentB))
  // {
  //   std::cout << "Invalid Encoder Output!" << std::endl;
  //   return;
  // }

  //Creates variables for change in each side info in inches (12.9590697 is circumference of wheel)
  DeltaL = ((double)(currentL - PreviousL) * 12.9590697) / tpr;
  DeltaR = ((double)(currentR - PreviousR) * 12.9590697) / tpr;
  DeltaB = ((double)(currentB - PreviousB) * 12.9590697) / tpr;
  
  Theta = Gyro.heading(degrees) / 180 * Pi;

  double DeltaAverage = (DeltaL + DeltaR) / 2.0;
  X += DeltaAverage * sin(Theta);
  Y += DeltaAverage * cos(Theta);
  X += DeltaB * sin(Theta + (Pi / 2.0));
  Y += DeltaB * cos(Theta + (Pi / 2.0));

  //Odom heading is converting the radian value of Theta into degrees
  OdomHeading = Theta * 57.295779513;

  //Converts values into newer values to allow for code to effectively work in next cycle
  PreviousL = currentL;
  PreviousR = currentR;
  PreviousB = currentB;

  /*--------------------GRAPHICS--------------------*/
    //Coordinates for each section of text
    int textadjustvalue = 55;
    int rowadjust = 39;

    //Sets graphical things for our display
    Brain.Screen.setPenWidth( 1 );
    vex::color redtile = vex::color( 210, 31, 60 );
    vex::color bluetile = vex::color( 14, 77, 146 );
    vex::color graytile = vex::color( 49, 51, 53 );
    Brain.Screen.setFillColor(vex::color( 0, 0, 0 ));
    Brain.Screen.setFont(vex::fontType::mono20);
    Brain.Screen.setPenColor( vex::color( 222, 49, 99 ) );

    //Displays all the field tiles, text of odom values, and a dot symbolizing the robot
    Brain.Screen.printAt(40,20 + textadjustvalue, "X-Pos:%f",-X);
    Brain.Screen.setPenColor( vex::color( 191, 10, 48 ) );
    Brain.Screen.printAt(40,50 + textadjustvalue, "Y-Pos:%f",Y);
    Brain.Screen.setPenColor( vex::color( 141, 2, 31 ) );
    Brain.Screen.printAt(40, 80 + textadjustvalue, "Theta:%f", Theta * 180 / Pi);
    Brain.Screen.setPenColor( vex::color( 83, 2, 1 ) );
    Brain.Screen.printAt(40,110 + textadjustvalue, "Angle:%f",X);
    Brain.Screen.setPenColor( vex::color( 255, 255, 255 ) );
    Brain.Screen.setFillColor( graytile );
    Brain.Screen.drawRectangle( 245, 2, 234, 234 );
    Brain.Screen.drawRectangle( 245, 2, 39, 39 );
    Brain.Screen.drawRectangle( 245, 80, 39, 39 );
    Brain.Screen.drawRectangle( 245, 119, 39, 39 );
    Brain.Screen.drawRectangle( 245, 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 197, 39, 39 );
    Brain.Screen.setFillColor( redtile );
    Brain.Screen.drawRectangle( 245, 158, 39, 39 );
    Brain.Screen.drawRectangle( 245, 41, 39, 39 );
    Brain.Screen.setFillColor( bluetile );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 158, 39, 39 );
    Brain.Screen.setPenColor( vex::color( 255,255,255));
    Brain.Screen.setFillColor( vex::color(0,0,0) );

    //This draws the robot body for position and arm for angle
    //double yfieldvalue = ((-Y)*fieldscale)+245-10;
    //double xfieldvalue = ((-X)*fieldscale)+245;
    double yfieldvalue = ((-Y)*fieldscale)+234+2;
    double xfieldvalue = ((-X)*fieldscale)+245;
    Brain.Screen.drawCircle(xfieldvalue, yfieldvalue, 10 );
    Brain.Screen.setPenWidth( 4 );
    //Line angle calculation:
    //x1 and y1 are the robot's coordinates, which in our case is xfieldvalue and yfieldvalue
    //angle is the angle the robot is facing, which in our case is Theta
    //(x1,y1, x1 + line_length*cos(angle),y1 + line_length*sin(angle)) = (x1,y1,x2,y2)
    Brain.Screen.drawLine(xfieldvalue, yfieldvalue, xfieldvalue+cos(-Theta-(Pi/2))*15, yfieldvalue+ sin(-Theta-(Pi/2)) *15);


  }
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void robotInit( void ) {

    vexcodeInit();
    Brain.resetTimer();

    RightFront.resetRotation();
    RightBack.resetRotation();
    LeftFront.resetRotation();
    LeftBack.resetRotation();

    EncoderC.resetRotation();
    EncoderE.resetRotation();
    EncoderG.resetRotation();

    //SET VALUES FOR INITIAL ROBOT POSITION
    X = 0;
    Y = 0;

    std::cout << "Starting Values:(" << EncoderC.position(degrees) << ", " << EncoderE.position(degrees) << ", " << EncoderG.position(degrees) << ")" << std::endl;
    
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {
  

}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  /*EncoderC.resetRotation();
  EncoderG.resetRotation();
  EncoderE.resetRotation();*/

  // DeltaL = 0;
  // DeltaR = 0;
  // DeltaB = 0;
  // currentL = 0;
  // currentR = 0;
  // currentB = 0;
  // PreviousL = 0;
  // PreviousR = 0;
  // PreviousB = 0;
  // DeltaTheta = 0;
  // DeltaXBack = 0;
  // DeltaYBack = 0;
  // X = 0;
  // Y = 0;
  // Theta = 0;
  // DeltaXSide = 0;
  // DeltaYSide = 0;
  // SideChord = 0;
  // BackChord = 0;
  // OdomHeading = 0;

  std::cout << "Initial Position: (" << X << ", " << Y << ") " << Theta << std::endl;
  std::cout << "Starting Encoder Values: (" << EncoderC.position(degrees) << ", " << EncoderE.position(degrees) << ", " << EncoderG.position(degrees) << ")" << std::endl;
  std::cout << "Initial Current Values: (" << currentL << ", " << currentR << ", " << currentB << ")" << std::endl;

  while (1){
    Brain.Screen.clearScreen();
    // std::cout << "Running? X: " << X << " Y: " << Y << " Theta: " << Theta << std::endl;

    //provides power to the motors to allow for movement of robot for testing using controller
    LeftBack.spin(vex::directionType::rev, Controller1.Axis3.value() + Controller1.Axis1.value(), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::rev, Controller1.Axis3.value() + Controller1.Axis1.value(), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, Controller1.Axis3.value() - Controller1.Axis1.value(), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, Controller1.Axis3.value() - Controller1.Axis1.value(), vex::velocityUnits::pct);

    //Calls the TrackPosition function
    TrackPOS();
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
  std::cout << "This probably never happens" << std::endl;
}
int main() {
    std::cout << "Entering main" << std::endl;
    robotInit();

    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function

    while(1) {
      vex::task::sleep(5); //Slight delay so the Brain doesn't overprocess
    }
}