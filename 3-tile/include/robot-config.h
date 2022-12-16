using namespace vex;

extern brain Brain;

// VEXcode devices
extern encoder RearEncoder;
extern encoder LeftEncoder;
extern encoder RightEncoder;
extern smartdrive Drivetrain;
extern motor_group flywheel;
extern motor rightMotorA;
extern motor rightMotorB;
extern motor leftMotorA;
extern motor leftMotorB;
extern inertial DrivetrainInertial;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );