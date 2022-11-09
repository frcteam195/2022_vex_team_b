using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightBack;
extern motor RightFront;
extern motor LeftFront;
extern motor LeftBack;
extern controller Controller1;
extern encoder EncoderC;
extern encoder EncoderE;
extern encoder EncoderG;
extern inertial Gyro;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );