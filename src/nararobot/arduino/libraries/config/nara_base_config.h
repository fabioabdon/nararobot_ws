#ifndef NARA_BASE_CONFIG_H
#define NARA_BASE_CONFIG_H

//uncomment the base you're building
#define NARA_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER      // 4WD robot
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
//#define USE_L298_DRIVER
#define USE_BTS7960_DRIVER
// #define USE_ESC

//uncomment the IMU you're using
//#define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define DEBUG 1

//#define K_P 0.6 // P constant
//#define K_I 0.3 // I constant
//#define K_D 0.5 // D constant

//define your robot' specs here
//#define MAX_RPM 330               // motor's maximum RPM
//#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
//#define WHEEL_DIAMETER 0.10       // wheel's diameter in meters
//#define PWM_BITS 8                // PWM Resolution of the microcontroller
//#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
//#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

//=================BIGGER ROBOT SPEC (BTS7960)=============================
#define K_P 0.05  // P constant
#define K_I 0.9   // I constant
#define K_D 0.1   // D constant

// define your robot' specs here
#define MAX_RPM 50               // motor's maximum RPM
#define COUNTS_PER_REV 2400      // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.336      // wheel's diameter in meters
#define PWM_BITS 8               // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.51  // distance between left and right wheels (0.575) (teste 2 com: 0.44)
#define FR_WHEELS_DISTANCE 0.46  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN
//================= END OF BIGGER ROBOT SPEC =============================

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

/// ENCODER PINS
#define MOTOR1_ENCODER_A 2
#define MOTOR1_ENCODER_B 3

#define MOTOR2_ENCODER_A 18
#define MOTOR2_ENCODER_B 19

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 26

//MOTOR PINS

#ifdef USE_BTS7960_DRIVER
  #define MOTOR_DRIVER BTS7960  

  #define MOTOR1_PWM 1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 8
  #define MOTOR1_IN_B 9

  #define MOTOR2_PWM 40 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 11 
  #define MOTOR2_IN_B 12

  #define MOTOR3_PWM 0 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_PWM 25 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 40
  #define MOTOR4_IN_B 24

  #define PWM_MAX 100
  #define PWM_MIN -PWM_MAX
#endif

#define STEERING_PIN 45

#endif
