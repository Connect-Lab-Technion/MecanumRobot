#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG 1

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 10 //hz
#define DEBUG_RATE 5

#define K_P 0.1 // P constant
#define K_I 0.2 // I constant
#define K_D 0.2 // D constant

// define your robot' specs here
#define MAX_RPM 366 // motor's maximum RPM
#define COUNTS_PER_REV 1560 // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.068 // wheel's diameter in meters
#define PWM_BITS 8 // PWM Resolution of the microcontroller
#define BASE_WIDTH 0.26 // width of the plate you are using

// ENCODER PINS
// left side encoders pins
#define MOTOR1_ENCODER_A 18 // front_A
#define MOTOR1_ENCODER_B 31 // front_B

#define MOTOR3_ENCODER_A 19 // rear_A
#define MOTOR3_ENCODER_B 38 // rear_B

// right side encoders pins
#define MOTOR2_ENCODER_A 3 // front_A
#define MOTOR2_ENCODER_B 49 // front_B

#define MOTOR4_ENCODER_A 2  // rear_A
#define MOTOR4_ENCODER_B A1 // rear_B

//left side motor pins
#define MOTOR1_PWM 11
#define MOTOR1_IN_A 34
#define MOTOR1_IN_B 35

#define MOTOR3_PWM  7
#define MOTOR3_IN_A 37
#define MOTOR3_IN_B 36

//right side motor pins
#define MOTOR2_PWM 4
#define MOTOR2_IN_A A4
#define MOTOR2_IN_B A5

#define MOTOR4_PWM 6
#define MOTOR4_IN_A 43
#define MOTOR4_IN_B 42
#endif
