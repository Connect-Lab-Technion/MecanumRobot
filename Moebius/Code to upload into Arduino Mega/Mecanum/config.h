#define ONBOARD_VER       "Mbs_onboard_v1.02"
#define BUILD_DATE        "2020-09-10"
#define DEBUG_ENABLE
//Motor param config
  #define MotorType         DC_Motor

//ROS publish speed
    #define IMU_PUBLISH_RATE 10 //hz
    #define VEL_PUBLISH_RATE 10 //hz
    #define COMMAND_RATE 10 //hz
    #define DEBUG_RATE 5

// define your robot' specs here
  #define MAX_PWM 255 // motor's maximum RPM
  #define MAX_RPM 330 // motor's maximum RPM
  #define COUNTS_PER_REV 2300 // wheel encoder's no of ticks per rev
  #define WHEEL_DIAMETER 0.068 // wheel's diameter in meters
  #define PWM_BITS 8 // PWM Resolution of the microcontroller
  #define BASE_WIDTH 0.26 // width of the plate you are using


#define a_PARAMETER          (0.311f)      
#define b_PARAMETER          (0.3075f)
