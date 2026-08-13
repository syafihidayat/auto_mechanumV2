// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define LED_PIN 13 // used for debugging status

// uncomment the base you're building
//  #define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
//  #define LINO_BASE SKID_STEER            // 4WD robot
#define LINO_BASE MECANUM // Mecanum drive robot

// uncomment the motor driver you're using
//  #define USE_GENERIC_2_IN_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
//  #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
#define USE_BTS7960_MOTOR_DRIVER // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

// uncomment the IMU you're using
//  #define USE_GY85_IMU                 90
//  #define USE_MPU6050_IMU              366.10
//  #define USE_MPU9150_IMU
//  #define USE_MPU9250_IMU
#define USE_BNO055_IMU

// #define K_P 70// 0          //49.5                           // P constant
// #define K_I 160.707317073//100        //141.428571429       I constant
// #define K_D 0                             // D constant

// #define K_P 70         //130 // 45 //1.2//55
// #define K_I 10                //310.101694915 // 80 //0.05 //0.045454545
// #define K_D 0

#define K_P 110         //130 // 45 //1.2//55
#define K_I 170                //310.101694915 // 80 //0.05 //0.045454545
#define K_D 0

#define K_P1 145
#define K_I1 320.428571429 
#define K_D1 0

// #define K_P2 0       // 45 //1.2//55
// #define K_I2 0 // 80 //0.05 //0.045454545
// #define K_D2 0


// #define K_P2 120          // 33
// #define K_I2 255.707317073 // 120
// #define K_D2 0

#define KP_LIFT 7                                                                         
#define KI_LIFT 4
#define KD_LIFT 0.01

#define KP_LIFT2 7                                                                        
#define KI_LIFT2 4
#define KD_LIFT2 0.01


// #define susan_kp 4.0
// #define susan_ki 0.0
// #define susan_kd 0.0

// motor 1 di pwm 60
// motor 2 di pwm 60
// motor 3 di pwm 61 1.60 rps
// motor 4 di pwm 60 1.53 rps

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

// define your robot' specs here
#define MOTOR_MAX_RPS 9.3               // 10.0 //11.2 rps                   // motor's max RPM
#define MAX_RPS_RATIO 0.9               // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 24      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24 // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 135             // 540   //350        // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 135          //96    // 540    //350      // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 135                    //1024            // 1024            // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 135               //1024            // 1024              // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.1              // 0.0985           // wheel's diameter in meters
#define ROBOT_DIAMETER 0.35             // 35cm           // distance between left and right wheels
#define ROBOT_RADIUS 0.175
#define PWM_BITS 8
// #define PWM_BITS 10
#define PWM_FREQUENCY 25000

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV true
#define MOTOR2_INV true
#define MOTOR3_INV true
#define MOTOR4_INV true

// ENCODER PINS
#define MOTOR1_ENCODER_A 34 //37
#define MOTOR1_ENCODER_B 35 //36

#define MOTOR2_ENCODER_A 39 //11
#define MOTOR2_ENCODER_B 38 //12

#define MOTOR3_ENCODER_A 40 //29
#define MOTOR3_ENCODER_B 41 //28

#define MOTOR4_ENCODER_A 19 //26
#define MOTOR4_ENCODER_B 18 //27

#ifdef USE_BTS7960_MOTOR_DRIVER

#define MOTOR1_PWM -1  // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR1_IN_A 22 //25 //22 // Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
#define MOTOR1_IN_B 23 //10 // Pin no 20 is not a PWM pin on Teensy 4.x, you can use pin no 0 instead.

#define MOTOR2_PWM -1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR2_IN_A 14 //19
#define MOTOR2_IN_B 15 //18

#define MOTOR3_PWM -1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR3_IN_A 37 //6
#define MOTOR3_IN_B 36 //5

#define MOTOR4_PWM -1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR4_IN_A 33 //3
#define MOTOR4_IN_B 29 //4

// #define MOTOR_upA 25
// #define MOTOR_upB 24

//========================================================================================================================
#define TOF_MIN_DIST 30
#define TOF_MAX_DIST 70                                    //parameter sensor tof
#define TOF_JUMP_MAX 100
//=========================================================================================================================

//=============================================================actuators==========================================================
// #define srv_elbow_pin 0
// #define srv_gripper_pin 0

// #define motor_susan_cw 0
// #define motor_susan_ccw 0

#define MOTOR_LIFT_CW_FRONT 28
#define MOTOR_LIFT_CCW_FRONT 25

#define MOTOR_LIFT_CW_BEHIND 12
#define MOTOR_LIFT_CCW_BEHIND 24

// #define MotorSlide_A 99
// #define MotorSlide_B 99

#define srvGripp 2

//================================================================================================================================


//============================================================sensors=============================================================
// #define proxy1 98
// #define proxy2 0

#define encoderex_x_A 6 //38
#define encoderex_x_B 7 //39

#define encoderex_y_A 8 //34
#define encoderex_y_B 9 //33

#define enca_lift_behind 20
#define encb_lift_behind 21

#define enca_lift_front 4
#define encb_lift_front 5

// #define IR_PIN 99

// #define limitRight 0
// #define limitLeft 1 

// #define limitlifterUp 99

#define solenoidHolder 45

// #define solenoidGripper 99

// #define limitlifter 99

#define limitSlideRight 3

// #define proxy 99

// #define susan_encoderA 0
// #define susan_encoderB 0

//================================================================================================================================


// IMU  16,17 / SDA1,SCL1
// ToF 19,18 / SDA0,SCL0
const int cw[6] = {
    MOTOR1_IN_A,
    MOTOR2_IN_A,
    MOTOR3_IN_A,
    MOTOR4_IN_A,
    MOTOR_LIFT_CW_FRONT,
    MOTOR_LIFT_CW_BEHIND};

const int ccw[6] = {
    MOTOR1_IN_B,
    MOTOR2_IN_B,
    MOTOR3_IN_B,
    MOTOR4_IN_B,
    MOTOR_LIFT_CCW_FRONT,
    MOTOR_LIFT_CCW_BEHIND};

// // encoder in array
// const int enca[4] = {
//     MOTOR1_ENCODER_A,
//     MOTOR2_ENCODER_A,
//     MOTOR3_ENCODER_A,
//     MOTOR4_ENCODER_A};
// const int encb[4] = {
//     MOTOR1_ENCODER_B,
//     MOTOR2_ENCODER_B,
//     MOTOR3_ENCODER_B,
//     MOTOR4_ENCODER_B};

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

struct but
{
    int A;
    int B;
    int X;
    int Y;
    int RT;
    int LT;
    int LB;
    int RB;
    int select;
    int start;
    int home;
    int up;
    int down;
    int left;
    int right;
} button;
struct joy
{
    double axis0_x;
    double axis0_y;
    double axis1_x;
    double axis1_y;
    int but_red;
    int but_blue;
    int but_black;
    int but_green;
} joystick;

#endif
#endif