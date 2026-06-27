#include <Arduino.h>

#include <stdio.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "odometry.h"
#include "config.h"
#include "kinematic.h"
#include "pid.h"
#include "imu.h"
#include "Servo.h"

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_VL53L0X.h>
#include <IRremote.hpp>

rcl_subscription_t twist_subscriber;
rcl_subscription_t button_sub;
// rcl_subscription_t proxy_data_sub;
rcl_subscription_t lifter_down_sub;
rcl_subscription_t start_descent_sub;
rcl_subscription_t tof_data_sub;
rcl_subscription_t descend_lifter_up_sub;
rcl_subscription_t allbutton;
rcl_subscription_t allow_lifter_up_sub;
rcl_subscription_t solenoidGripper_sub;
rcl_subscription_t gui_start_sub;
rcl_subscription_t lifter_grid_sub;
rcl_subscription_t lifter_entry_sub;
rcl_subscription_t lifter_behind_entry_sub;
rcl_subscription_t lifter_front_down_sub;
rcl_subscription_t lifter_behind_down_sub;

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t checking_input;
rcl_publisher_t limit_publisher;
rcl_publisher_t limit_slide_publisher;
rcl_publisher_t tof_publisher;
rcl_publisher_t lifter_down2_publisher;
rcl_publisher_t robot_up_publisher;
rcl_publisher_t infra_publisher;
rcl_publisher_t after_climb_publisher;
rcl_publisher_t wait_lifter_publisher;
rcl_publisher_t descend_lifter_up_publisher;
rcl_publisher_t ir_code_publisher;
rcl_publisher_t bluePill_status_publisher;
rcl_publisher_t front_homing_done_pub;
rcl_publisher_t behind_homing_done_pub;

std_msgs__msg__Int8 button_msg;
std_msgs__msg__Int8 allbutton_msg;
std_msgs__msg__Float32MultiArray checking_input_msg;
std_msgs__msg__UInt8MultiArray bluePill_status_msg;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Bool proxy_data_msg;
std_msgs__msg__Bool lifter_down_msg;
std_msgs__msg__Bool limit_data_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__UInt16 tof_msg;
std_msgs__msg__Bool bool_msg;
std_msgs__msg__Bool wait_msg;
std_msgs__msg__Bool start_descent_msg;
std_msgs__msg__Bool descend_lifter_up_msg;
std_msgs__msg__Bool allow_lifter_up_msg;
std_msgs__msg__Bool solenoidGripper_msg;
std_msgs__msg__Bool gui_start_msg;
std_msgs__msg__Int8 lifter_grid_msg;
std_msgs__msg__Bool lifter_entry_msg;
std_msgs__msg__Bool lifter_behind_entry_msg;
std_msgs__msg__Bool front_homing_done_msg;
std_msgs__msg__Bool behind_homing_done_msg;
std_msgs__msg__Bool lifter_front_down_msg;
std_msgs__msg__Bool lifter_behind_down_msg;

// std_msgs__msg__UInt16 UInt16_msg;
// std_msgs__msg__UInt8 UInt8_msg;
// std_msgs__msg__Int8 int_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

void setMotor(int cwPin, int ccwPin, float pwmVal);
void moveBase();
void publishData();
void proxyPublish();
void limitPublish();
void bluePill_publish();
void publish_tof();
void pubInfraredTrans();
void control_pos();
void control_pos_lifter_behind();
void control_pos_lifter_front();
void start_grip_sequence();
void runStartSequence();
void readBluePillSerial();
void homingLifterBlocking();
void handle_front_homing_loop();
void handle_behind_homing_loop();
void sendSlideCommand(uint8_t target);
// void limitLifter(int target);
// void limitLifter();
void limitMotor(int target);
// void proxy_data_callback(const void *msgin);
void twistCallback(const void *msgin);
void allbuttonCallback(const void *msgin);
void lifter_callback(const void *msgin);
void start_descent_callback(const void *msgin);
void descend_lifter_up_callback(const void *msgin);
void allow_lifter_up_callback(const void *msgin);
void solenoid_grip_callback(const void *msgin);
void gui_start_callback(const void *msgin);
void lifter_grid_callback(const void *msgin);
void lifter_entry_callback(const void *msgin);
void lifter_behind_entry_callback(const void *msgin);
void lifter_front_down_callback(const void *msgin);
void lifter_behind_down_callback(const void *msgin);
void syncTime();
void error_loop();
struct timespec getTime();
bool createEntities();
bool destroyEntities();
void flashLED(int n_times);
template <int j>
void readEncoder();
bool update_tof();

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long prevT = 0;
uint32_t state_timer = 0;

unsigned long last_sequence = 0;
unsigned long last_publishData = 0;
unsigned long last_tof = 0;
unsigned long last_receiver = 0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

enum GripStep
{
  GRIP_IDLE,
  LIFTER_DOWN,
  GRIPPER_CLOSE,
  GRIPPER_CLOSE_2,
  LIFTER_DOWN_2,
  WAIT_MOTOR_HOMING,
  WAIT_MOTOR_HOMING2,
  MOTOR_SLIDE,
  SOL_HOLDER,
  OPEN_SOL_HOLDER,
  WAIT_AFTER_DOWN,
  WAIT_SENSOR_READY,
  WAIT_LIFTER_DOWN,
  WAIT_GRIPPER_OPEN,
  WAIT_GRIPPER_CLOSE,
  GRIPPER_OPEN,
  LIFTER_UP
};

enum Mode
{
  NORMAL,
  CLIMBING,
  DESCENDING,
  RECOVERY
};

enum BluePillRxState
{
  BP_WAIT_SYNC1,
  BP_WAIT_SYNC2,
  BP_WAIT_DATA,
  BP_WAIT_TOF_STATUS,
  BP_WAIT_TOF_MSB,
  BP_WAIT_TOF_LSB
};

BluePillRxState bluePillRxState = BP_WAIT_SYNC1;

Mode mode = NORMAL;

unsigned long grip_timer = 0;
GripStep grip_step = GRIP_IDLE;
volatile bool proxy_detected = true;
static bool sensor_ready = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
// Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int enca[8] = {MOTOR1_ENCODER_A, MOTOR2_ENCODER_A, MOTOR3_ENCODER_A, MOTOR4_ENCODER_A, encoderex_x_A, encoderex_y_A, enca_lift_front, enca_lift_behind};
const int encb[8] = {MOTOR1_ENCODER_B, MOTOR2_ENCODER_B, MOTOR3_ENCODER_B, MOTOR4_ENCODER_B, encoderex_x_B, encoderex_y_B, encb_lift_front, encb_lift_behind};

bool limitState[4] = {true, true, true, true};
bool proxyState[4] = {true, true, true, true};

volatile long pos[8];

PID wheel1(PWM_MIN, PWM_MAX, K_P1, K_I1, K_D1);
PID wheel2(PWM_MIN, PWM_MAX, K_P1, K_I1, K_D1);
PID wheel3(PWM_MIN, PWM_MAX, K_P1, K_I1, K_D1);
PID wheel4(PWM_MIN, PWM_MAX, K_P1, K_I1, K_D1);
PID external_encoder1(0, 0, 0, 0, 0);
PID external_encoder2(0, 0, 0, 0, 0);
PID external_encoder3(PWM_MIN, PWM_MAX, KP_LIFT, KI_LIFT, KD_LIFT);
PID external_encoder4(PWM_MIN, PWM_MAX, KP_LIFT2, KI_LIFT2, KD_LIFT2);

Kinematic Kinematics(
    Kinematic::LINO_BASE,
    MOTOR_MAX_RPS,
    MAX_RPS_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    ROBOT_DIAMETER);

Odometry odometry;
IMU imu_sensor;
Servo srv;
bool homed = false;
bool homed_lifter = false;
bool tof_ok = false;
bool solenoid_triggered = false;
uint16_t tof_distance = 0;
bool tof_valid = false;

bool startSeqTriggered = false;
bool startSeqActive = false;
bool startSeqDone = false;

bool stage2_grid_active = false;
float stage2_target_front = 2650;
float stage2_target_behind = 2650;

bool trigger_front_homing = false;
bool trigger_behind_homing = false;

void setup()
{
  // Serial1.begin(9600);
  Serial1.begin(115200);

  // Serial.begin(115200);

  set_microros_serial_transports(Serial);

  while (!imu_sensor.init())
  {
    flashLED(3);
  }

  for (int i = 0; i < 9; i++)
  {

    pinMode(cw[i], OUTPUT);
    pinMode(ccw[i], OUTPUT);

    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);

    analogWriteFrequency(cw[i], PWM_FREQUENCY);
    analogWriteFrequency(ccw[i], PWM_FREQUENCY);

    analogWriteResolution(PWM_BITS);
    analogWrite(cw[i], 0);
    analogWrite(ccw[i], 0);
  }

  // pinMode(proxy1, INPUT);
  srv.attach(srvGripp);

  pinMode(limitSlideRight, INPUT_PULLUP);

  external_encoder1.ppr_total(2048);
  external_encoder2.ppr_total(2048);
  // external_encoder3.ppr_total(975);
  external_encoder3.ppr_total(1024);
  external_encoder4.ppr_total(1024);

  wheel1.ppr_total(COUNTS_PER_REV1);
  wheel2.ppr_total(COUNTS_PER_REV2);
  wheel3.ppr_total(COUNTS_PER_REV3);
  wheel4.ppr_total(COUNTS_PER_REV4);

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder<4>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder<5>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[6]), readEncoder<6>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[7]), readEncoder<7>, RISING);

  homingLifterBlocking();

  srv.write(150);

  pinMode(LED_PIN, OUTPUT);
}

bool last_proxy = false;
bool proxy_latched = false;
bool disable_tof_trigger = false;

void loop()
{

  readBluePillSerial();

  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroyEntities();
      for (int i = 0; i < 9; i++)
      {
        pos[i] = 0;
      }
    }
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

      // if (grip_step == WAIT_SENSOR_READY && !disable_tof_trigger)
      // {

      //   bool tof_detected = (tof_distance < TOF_MAX_DIST && tof_distance > TOF_MIN_DIST);
      //   if (tof_detected && !last_proxy && !proxy_latched)

      //   {
      //     grip_step = WAIT_AFTER_DOWN;
      //     grip_timer = millis();
      //     sensor_ready = true;
      //     proxy_latched = true;
      //   }

      //   last_proxy = tof_detected;
      // }

      unsigned long now = millis();

      if (now - last_publishData >= 20)
      {
        last_publishData = now;
        publishData();
      }

      if (now - last_sequence >= 50)
      {
        last_sequence = now;
        limitPublish();
        bluePill_publish();

        publish_tof();
        // pubInfraredTrans();
      }
      handle_front_homing_loop();
      handle_behind_homing_loop();

      start_grip_sequence();
      runStartSequence();
      moveBase();
      // update_tof();

      // proxyPublish();
    }
    break;
  case AGENT_DISCONNECTED:
    destroyEntities();

    setMotor(cw[0], ccw[0], 0);
    setMotor(cw[1], ccw[1], 0);
    setMotor(cw[2], ccw[2], 0);
    setMotor(cw[3], ccw[3], 0);
    // digitalWrite(solenoidGripper, LOW);
    state = WAITING_AGENT;
    break;
  default:
    break;
  }
}

bool slide_done = false;
bool ir_button_a = false;
bool ir_button_b = false;

void readBluePillSerial()
{

  static uint8_t tof_status = 0;
  static uint8_t tof_msb = 0;

  while (Serial1.available())
  {
    uint8_t b = Serial1.read();

    switch (bluePillRxState)
    {
    case BP_WAIT_SYNC1:
      if (b == 0xAA)
        bluePillRxState = BP_WAIT_SYNC2;
      break;

    case BP_WAIT_SYNC2:
      if (b == 0x55)
        bluePillRxState = BP_WAIT_DATA;
      else if (b != 0xAA)
        bluePillRxState = BP_WAIT_SYNC1;
      break;

    case BP_WAIT_DATA:
    {
      if (b == 0xFE) // ID khusus TOF
      {
        bluePillRxState = BP_WAIT_TOF_STATUS;
      }
      else
      {
        // paket limit/proxy seperti biasa
        uint8_t id = b >> 1;
        bool value = b & 0x01;

        if (id <= 3)
          limitState[id] = value;
        else if (id <= 7)
          proxyState[id - 4] = value;
        else if (id == 8)
          slide_done = value;
        else if (id == 67)
          ir_button_a = value;
        else if (id == 68)
        {
          ir_button_b = value;

          if (value)
          {
            bool_msg.data = true;
            RCSOFTCHECK(rcl_publish(&infra_publisher, &bool_msg, NULL));
          }
        }

        bluePillRxState = BP_WAIT_SYNC1;
      }
      break;
    }

    case BP_WAIT_TOF_STATUS:
      tof_status = b; // 1 = valid dalam range, 0 = tidak
      bluePillRxState = BP_WAIT_TOF_MSB;
      break;

    case BP_WAIT_TOF_MSB:
      tof_msb = b;
      bluePillRxState = BP_WAIT_TOF_LSB;
      break;

    case BP_WAIT_TOF_LSB:
      tof_distance = ((uint16_t)tof_msb << 8) | b;
      tof_valid = (tof_status == 1);
      bluePillRxState = BP_WAIT_SYNC1;
      break;
    }
  }

  // while (Serial1.available())
  // {
  //   uint8_t b = Serial1.read();

  //   switch (bluePillRxState)
  //   {
  //   case BP_WAIT_SYNC1:
  //     if (b == 0xAA)
  //       bluePillRxState = BP_WAIT_SYNC2;
  //     break;

  //   case BP_WAIT_SYNC2:
  //     if (b == 0x55)
  //       bluePillRxState = BP_WAIT_DATA;
  //     else if (b != 0xAA)
  //       bluePillRxState = BP_WAIT_SYNC1; // bukan sync2 valid, reset
  //     // kalau b == 0xAA lagi, tetap di BP_WAIT_SYNC2 (anggap sync1 baru)
  //     break;

  //   case BP_WAIT_DATA:
  //   {
  //     uint8_t id = b >> 1;
  //     bool value = b & 0x01;

  //     if (id <= 3)
  //       limitState[id] = value; // id 0-3 -> LIMIT1-4
  //     else if (id <= 7)
  //       proxyState[id - 4] = value; // id 4-7 -> PROXY1L/R, PROXY2L/R

  //     bluePillRxState = BP_WAIT_SYNC1; // siap untuk frame berikutnya
  //     break;
  //   }
  //   }
  // }
}

#define LIMIT_FRONT_IDX 0
#define LIMIT_BEHIND_IDX 1

void homingLifterBlocking()
{
  bool homedFront = false;
  bool homedBehind = false;
  unsigned long startTime = millis();

  while (!homedFront || !homedBehind)
  {
    readBluePillSerial(); // wajib! limitState[] cuma update kalau ini dipanggil terus

    // --- Lifter depan ---
    if (!homedFront)
    {
      if (limitState[LIMIT_FRONT_IDX] == false) // 0 = tertekan = sudah di limit
      {
        setMotor(MOTOR_LIFT_CW_FRONT, MOTOR_LIFT_CCW_FRONT, 0);
        pos[6] = 0;
        homedFront = true;
      }
      else
      {
        setMotor(MOTOR_LIFT_CW_FRONT, MOTOR_LIFT_CCW_FRONT, -200); // arah/PWM sesuaikan
      }
    }

    // --- Lifter belakang ---
    if (!homedBehind)
    {
      if (limitState[LIMIT_BEHIND_IDX] == false)
      {
        setMotor(MOTOR_LIFT_CW_BEHIND, MOTOR_LIFT_CCW_BEHIND, 0);
        pos[7] = 0;
        homedBehind = true;
      }
      else
      {
        setMotor(MOTOR_LIFT_CW_BEHIND, MOTOR_LIFT_CCW_BEHIND, -200); // arah/PWM sesuaikan
      }
    }

    // Safety timeout — kalau lebih dari 8 detik belum ketemu limit, stop biar gak stall
    // if (millis() - startTime > 8000)
    // {
    //   setMotor(MOTOR_LIFT_CW_FRONT, MOTOR_LIFT_CCW_FRONT, 0);
    //   setMotor(MOTOR_LIFT_CW_BEHIND, MOTOR_LIFT_CCW_BEHIND, 0);
    //   break; // keluar loop walau belum homed, supaya gak nyangkut selamanya
    // }

    delay(10);
  }

  homed_lifter = true;
  // homed_lifter = homedFront && homedBehind;
}

float toLinear(double pos, float radius)
{
  return pos * radius;
}

unsigned long pos_prevT = 0;
unsigned long pos_prevT2 = 0;
// void control_pos(float angle, float pwm)
// {
//   unsigned long pos_currT = micros();
//   float deltaT = ((float)(pos_currT - pos_prevT)) / 1.0e6;
//   float pos_controlled = external_encoder3.control_count(angle, pos[6], pwm, deltaT);
//   setMotor(MOTOR_LIFT_CW, MOTOR_LIFT_CCW, pos_controlled);

//   pos_prevT = pos_currT;
// }

void control_pos_lifter_behind(float angle, float pwm)
{
  unsigned long pos_currT = micros();
  float deltaT = ((float)(pos_currT - pos_prevT)) / 1.0e6;

  if (deltaT <= 0 || deltaT > 1.0)
  {
    pos_prevT = pos_currT;
    return;
  }

  float current_pos = -pos[7];
  float error = angle - current_pos;

  if (fabs(error) < 8)
  {
    setMotor(MOTOR_LIFT_CW_BEHIND, MOTOR_LIFT_CCW_BEHIND, 0);
    pos_prevT = pos_currT;
    return;
  }

  float pos_controlled = external_encoder3.control_count(angle, current_pos, pwm, deltaT);
  setMotor(MOTOR_LIFT_CW_BEHIND, MOTOR_LIFT_CCW_BEHIND, pos_controlled);
  pos_prevT = pos_currT;
}

void control_pos_lifter_front(float angle, float pwm)
{
  unsigned long pos_currT = micros();
  float deltaT = ((float)(pos_currT - pos_prevT2)) / 1.0e6;

  if (deltaT <= 0 || deltaT > 1.0)
  {
    pos_prevT2 = pos_currT;
    return;
  }

  float current_pos = pos[6]; // ← simpan dulu, konsisten dipakai semua
  float error = angle - current_pos;

  if (fabs(error) < 10)
  {
    setMotor(MOTOR_LIFT_CW_FRONT, MOTOR_LIFT_CCW_FRONT, 0);
    pos_prevT2 = pos_currT;
    return;
  }

  float controlled = external_encoder4.control_count(angle, current_pos, pwm, deltaT); // ← pakai current_pos
  setMotor(MOTOR_LIFT_CW_FRONT, MOTOR_LIFT_CCW_FRONT, controlled);
  pos_prevT2 = pos_currT;
}

bool lifter_locked = false;
bool limit_sent = false;
bool after_climb_sent = false;
bool allow_lifter_up_granted = false;
bool was_descend = false;
bool was_descend_done = false;
bool descend_command_received = false;
bool descend_lifter_up_done = true;
bool descend_done_sent = false;

void moveBase()
{

  sensors_event_t event, angVelocityData;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  unsigned long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;

  if (((millis() - prev_cmd_time) >= 200))
  {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    digitalWrite(LED_PIN, HIGH);
  }

  Kinematic::rps req_rps;
  req_rps = Kinematics.getRPS(
      twist_msg.linear.x,
      twist_msg.linear.y * -1,
      twist_msg.angular.z,
      event.orientation.x);

  float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
  float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
  float controlled_motor3 = -wheel3.control_speed(req_rps.motor3, -pos[2], deltaT);
  float controlled_motor4 = -wheel4.control_speed(req_rps.motor4, -pos[3], deltaT);

  float current_rps1 = wheel1.get_filt_vel();
  float current_rps2 = wheel2.get_filt_vel();
  float current_rps3 = wheel3.get_filt_vel();
  float current_rps4 = wheel4.get_filt_vel();

  if (fabs(req_rps.motor1) < 0.02)
  {
    controlled_motor1 = 0.0;
  }
  if (fabs(req_rps.motor2) < 0.02)
  {
    controlled_motor2 = 0.0;
  }
  if (fabs(req_rps.motor3) < 0.02)
  {
    controlled_motor3 = 0.0;
  }
  if (fabs(req_rps.motor4) < 0.02)
  {
    controlled_motor4 = 0.0;
  }

  if (startSeqActive)
  {
    setMotor(cw[0], ccw[0], 0);
    setMotor(cw[1], ccw[1], 0);
    setMotor(cw[2], ccw[2], 0);
    setMotor(cw[3], ccw[3], 0);
  }
  else
  {
    setMotor(cw[0], ccw[0], controlled_motor1);
    setMotor(cw[1], ccw[1], controlled_motor2);
    setMotor(cw[2], ccw[2], controlled_motor3);
    setMotor(cw[3], ccw[3], controlled_motor4);
  }

  Kinematic::velocities vel = Kinematics.getVelocities(
      current_rps1,
      current_rps2,
      current_rps3,
      current_rps4);

  float vel_enc1 = external_encoder1.convert_speed(pos[4], deltaT);
  float vel_enc2 = external_encoder2.convert_speed(pos[5], deltaT);

  float vx_ext = toLinear(vel_enc1, 0.02375);
  float vy_ext = toLinear(vel_enc2, 0.02375);

  float yaw = event.orientation.z * (M_PI / 180.0);
  float pitch = event.orientation.y;

  if (fabs(angVelocityData.gyro.z) > 0.25)
  {
    vx_ext = 0;
    vy_ext = 0;
  }

  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(
      vel_dt,
      vx_ext,
      vy_ext,
      angVelocityData.gyro.z,
      yaw);

  // unsigned long now = millis();
  // float vel_dt = (now - prev_odom_update) / 1000.0;
  // prev_odom_update = now;
  // odometry.update(
  // 	vel_dt,
  // 	vel.linear_x,
  // 	vel.linear_y,
  // 	vel.angular_z);

  prevT = currT;

  checking_input_msg.data.data[0] = current_rps1;
  checking_input_msg.data.data[1] = current_rps2;
  checking_input_msg.data.data[2] = current_rps3;
  checking_input_msg.data.data[3] = current_rps4;
  checking_input_msg.data.data[4] = event.orientation.x;
  checking_input_msg.data.data[5] = event.orientation.y;
  checking_input_msg.data.data[6] = pos[6];
  checking_input_msg.data.data[7] = pos[7];

  unsigned long now_mb = millis();
  if (now_mb - last_tof >= 50)
  {
    last_tof = now_mb;
    // proxy_data_msg.data = proxyDetected;
    RCSOFTCHECK(rcl_publish(&checking_input, &checking_input_msg, NULL));
  }

  // RCSOFTCHECK(rcl_publish(&checking_input, &checking_input_msg, NULL));

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

void publishData()
{

  odom_msg = odometry.getData();
  imu_msg = imu_sensor.getData();

  struct timespec time_stamp = getTime();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void sendSlideCommand(uint8_t target)
{
  Serial1.write(0xBB);
  Serial1.write(0x44);
  Serial1.write(target == 0 ? 0x01 : 0x02);
}

void sendSolenoidCommand(uint8_t activate)
{
  Serial1.write(0xBB);
  Serial1.write(0x44);
  Serial1.write(activate ? 0x03 : 0x04);
}

void publish_tof()
{
  if (!tof_valid)
    return;

  tof_msg.data = tof_distance;

  RCSOFTCHECK(rcl_publish(&tof_publisher, &tof_msg, NULL))
}

void limitPublish()

{
  int state_right = digitalRead(limitSlideRight);

  bool_msg.data = (state_right == LOW);

  RCSOFTCHECK(rcl_publish(&limit_slide_publisher, &bool_msg, NULL))
}

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "hardware_node", "", &support));

  executor = rclc_executor_get_zero_initialized_executor();

  RCCHECK(rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "omni_cont/cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &allbutton,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "allbutton"));

  // RCCHECK(rclc_subscription_init_default(
  //     &proxy_data_sub,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  //     "/true_sensor_proxy"));

  RCCHECK(rclc_subscription_init_default(
      &lifter_down_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_control"));

  RCCHECK(rclc_subscription_init_default(
      &gui_start_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "robot_start"));

  RCCHECK(rclc_subscription_init_default(
      &lifter_grid_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "lifter_grid_control"));

  RCCHECK(rclc_subscription_init_default(
      &lifter_entry_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_entry_stage2"));

  RCCHECK(rclc_subscription_init_default(
      &lifter_behind_entry_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_behind_stage2"));

  RCCHECK(rclc_subscription_init_default(
      &lifter_front_down_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_front_down"));

  RCCHECK(rclc_subscription_init_default(
      &lifter_behind_down_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_behind_down"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 9, &allocator));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &twist_subscriber,
      &twist_msg,
      &twistCallback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &allbutton,
      &allbutton_msg,
      &allbuttonCallback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &lifter_down_sub,
      &lifter_down_msg,
      &lifter_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &gui_start_sub,
      &gui_start_msg,
      &gui_start_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &lifter_grid_sub,
      &lifter_grid_msg,
      &lifter_grid_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &lifter_entry_sub,
      &lifter_entry_msg,
      &lifter_entry_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &lifter_behind_entry_sub,
      &lifter_behind_entry_msg,
      &lifter_behind_entry_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &lifter_front_down_sub,
      &lifter_front_down_msg,
      &lifter_front_down_callback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &lifter_behind_down_sub,
      &lifter_behind_down_msg,
      &lifter_behind_down_callback,
      ON_NEW_DATA));

  // RCCHECK(rclc_executor_add_subscription(
  //     &executor,
  //     &start_descent_sub,
  //     &start_descent_msg,
  //     &start_descent_callback,
  // ON_NEW_DATA));

  // RCCHECK(rclc_executor_add_subscription(
  //     &executor,
  //     &allow_lifter_up_sub,
  //     &allow_lifter_up_msg,
  //     &allow_lifter_up_callback,
  //     ON_NEW_DATA));

  // RCCHECK(rclc_executor_add_subscription(
  //     &executor,
  //     &solenoidGripper_sub,
  //     &solenoidGripper_msg,
  //     &solenoid_grip_callback,
  //     ON_NEW_DATA));

  RCCHECK(rclc_publisher_init_default(
      &checking_input,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "checking_input"));
  checking_input_msg.data.data = (float *)malloc(8 * sizeof(float)); // Sesuaikan jumlah elemen
  checking_input_msg.data.size = 8;
  checking_input_msg.data.capacity = 8;

  RCCHECK(rclc_publisher_init_default(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu_extern/data"));

  RCCHECK(rclc_publisher_init_default(
      &limit_slide_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "limitSlideData"));

  RCCHECK(rclc_publisher_init_default(
      &robot_up_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "robot_up_wp0"));

  RCCHECK(rclc_publisher_init_default(
      &tof_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
      "tof_distance"));

  RCCHECK(rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom/unfiltered"));

  RCCHECK(rclc_publisher_init_default(
      &bluePill_status_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
      "bluePill_data"));

  RCCHECK(rclc_publisher_init_default(
      &infra_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "infraReceive"));

  RCCHECK(rclc_publisher_init_default(
      &lifter_down2_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_down2"));

  RCCHECK(rclc_publisher_init_default(
      &front_homing_done_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_front_done"));

  RCCHECK(rclc_publisher_init_default(
      &behind_homing_done_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "lifter_behind_done"));

  // RCCHECK(rclc_publisher_init_default(
  //     &after_climb_publisher,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  //     "afterClimb"));

  // RCCHECK(rclc_publisher_init_default(
  //     &descend_lifter_up_publisher,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg, Bool),
  //     "/descend_lifter_up_after_down"));

  syncTime();
  digitalWrite(LED_PIN, HIGH);
  return true;
}

bool destroyEntities()
{

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
  RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
  RCCHECK(rcl_publisher_fini(&infra_publisher, &node));
  RCCHECK(rcl_publisher_fini(&checking_input, &node));
  RCCHECK(rcl_publisher_fini(&bluePill_status_publisher, &node));
  // RCCHECK(rcl_publisher_fini(&limit_publisher, &node));
  RCCHECK(rcl_publisher_fini(&limit_slide_publisher, &node));
  // RCCHECK(rcl_publisher_fini(&descend_lifter_up_publisher, &node));
  // RCCHECK(rcl_publisher_fini(&after_climb_publisher, &node));
  RCCHECK(rcl_publisher_fini(&lifter_down2_publisher, &node));
  RCCHECK(rcl_publisher_fini(&tof_publisher, &node));
  RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));

  // RCCHECK(rcl_publisher_fini(&wait_lifter_publisher, &node));
  // RCCHECK(rcl_publisher_fini(&ir_code_publisher, &node));

  RCCHECK(rcl_subscription_fini(&allbutton, &node));
  RCCHECK(rcl_subscription_fini(&lifter_down_sub, &node));
  RCCHECK(rcl_subscription_fini(&lifter_grid_sub, &node));
  RCCHECK(rcl_subscription_fini(&lifter_entry_sub, &node));
  RCCHECK(rcl_subscription_fini(&lifter_behind_entry_sub, &node));
  // RCCHECK(rcl_subscription_fini(&start_descent_sub, &node));
  // RCCHECK(rcl_subscription_fini(&allow_lifter_up_sub, &node));
  // RCCHECK(rcl_subscription_fini(&solenoidGripper_sub, &node));

  RCCHECK(rcl_node_fini(&node));
  // RCCHECK(rcl_timer_fini(&control_timer));
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  digitalWrite(LED_PIN, HIGH);

  return true;
}

void syncTime()
{
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();

  time_offset = ros_time_ms - now;
}

void error_loop()
{
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(100);
}

void twistCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  twist_msg = *msg;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  prev_cmd_time = millis();
}

bool lifter_triggered = false;

void lifter_callback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data)
  {

    if (grip_step != GRIP_IDLE || lifter_triggered)
      return;

    lifter_triggered = true;
    grip_step = LIFTER_DOWN;
    sensor_ready = false;
    grip_timer = millis();
    stage2_grid_active = false;
  }
}

void gui_start_callback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;

  if (!msg->data)
    return;

  if (startSeqTriggered)
    return;

  startSeqTriggered = true;
  startSeqActive = true;
  startSeqDone = false;
}

void runStartSequence()
{
  if (!startSeqActive)
    return;

  static unsigned long startSeqTimer = 0;
  if (startSeqTimer == 0)
    startSeqTimer = millis();

  control_pos_lifter_front(241, 200);
  control_pos_lifter_behind(205, 200);

  bool frontReady = fabs(241 - pos[6]) < 8;
  bool behindReady = fabs(205 - (-pos[7])) < 8;
  bool timeout = (millis() - startSeqTimer) >= 2000; // 5 detik

  if (frontReady && behindReady || timeout)
  {
    startSeqActive = false;
    startSeqDone = true;
    startSeqTimer = 0; // reset untuk pemakaian berikutnya

    wheel1.reset();
    wheel2.reset();
    wheel3.reset();
    wheel4.reset();

    std_msgs__msg__Bool msg;
    msg.data = true;
    RCSOFTCHECK(rcl_publish(&robot_up_publisher, &msg, NULL));
  }
}

void lifter_grid_callback(const void *msgin)
{
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  int8_t col = msg->data;

  if (col == 0)
  {
    stage2_target_front = 3888;
    stage2_target_behind = 3888;
  }
  else if (col == 1)
  {
    stage2_target_front = 1899;
    stage2_target_behind = 1899;
  }
  else if (col == 2)
  {
    stage2_target_front = 3888;
    stage2_target_behind = 3888;
  }

  stage2_grid_active = true;
}

void lifter_entry_callback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data)
  {
    trigger_front_homing = true;
  }
}

void lifter_behind_entry_callback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data)
  {
    trigger_behind_homing = true;
  }
}

void handle_front_homing_loop()
{
  if (!trigger_front_homing)
    return;

  if (limitState[LIMIT_FRONT_IDX] == false)
  {
    setMotor(MOTOR_LIFT_CW_FRONT, MOTOR_LIFT_CCW_FRONT, 0);
    pos[6] = 0;
    trigger_front_homing = false;

    lifter_triggered = false;
    stage2_grid_active = false;

    front_homing_done_msg.data = true;
    rcl_publish(&front_homing_done_pub, &front_homing_done_msg, NULL);
  }
  else
  {
    setMotor(MOTOR_LIFT_CW_FRONT, MOTOR_LIFT_CCW_FRONT, -200);

    front_homing_done_msg.data = false;
    rcl_publish(&front_homing_done_pub, &front_homing_done_msg, NULL);
  }
}

void handle_behind_homing_loop()
{
  if (!trigger_behind_homing)
    return;

  if (limitState[LIMIT_BEHIND_IDX] == false) // limit sudah ketekan = SELESAI
  {
    setMotor(MOTOR_LIFT_CW_BEHIND, MOTOR_LIFT_CCW_BEHIND, 0);
    pos[7] = 0;
    trigger_behind_homing = false;

    lifter_triggered = false;
    stage2_grid_active = false;

    behind_homing_done_msg.data = true;
    rcl_publish(&behind_homing_done_pub, &behind_homing_done_msg, NULL);
  }
  else // masih proses naik
  {
    setMotor(MOTOR_LIFT_CW_BEHIND, MOTOR_LIFT_CCW_BEHIND, -200);

    behind_homing_done_msg.data = false; // ✅ penting: jangan ulangi bug yang sama!
    rcl_publish(&behind_homing_done_pub, &behind_homing_done_msg, NULL);
  }
}

bool front_down_only = false;
bool behind_down_only = false;

void lifter_front_down_callback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data)
  {
    stage2_target_front = 200; // turun sedikit
    // stage2_grid_active = true;   // aktifkan control pos
    front_down_only = true;
  }
}

void lifter_behind_down_callback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data)
  {
    stage2_target_behind = 200;
    behind_down_only = true;
  }
}

uint8_t bluePill_data_buff[8];

void bluePill_publish()
{
  for (int i = 0; i < 4; i++)
    bluePill_data_buff[i] = limitState[i];
  for (int i = 0; i < 4; i++)
    bluePill_data_buff[i + 4] = proxyState[i];

  bluePill_status_msg.data.data = bluePill_data_buff;
  bluePill_status_msg.data.size = 8;
  bluePill_status_msg.data.capacity = 8;

  RCSOFTCHECK(rcl_publish(&bluePill_status_publisher, &bluePill_status_msg, NULL));
}

void start_grip_sequence()
{

  if (mode == CLIMBING || mode == DESCENDING)
    return;

  if (grip_step == GRIP_IDLE)
  {
    if (lifter_triggered)
    {
      if (!trigger_front_homing)
      {
        control_pos_lifter_front(1890, 200);
      }
      if (!trigger_behind_homing)
      {
        control_pos_lifter_behind(1890, 245);
      }
    }
    else if (front_down_only || behind_down_only)
    {
      if (front_down_only && !trigger_front_homing)
      {
        control_pos_lifter_front(200, 200);
      }
      if (behind_down_only && !trigger_behind_homing)
      {
        control_pos_lifter_behind(200, 245);
      }
    }
    else if (stage2_grid_active)
    {
      if (!trigger_front_homing)
      {
        control_pos_lifter_front(stage2_target_front, 200);
      }

      if (!trigger_behind_homing)
      {
        control_pos_lifter_behind(stage2_target_behind, 245);
      }
    }
    return;
  }

  switch (grip_step)
  {

  case WAIT_LIFTER_DOWN:

    if (millis() - grip_timer >= 1600)
    {
      grip_timer = millis();
      disable_tof_trigger = false;
      grip_step = LIFTER_DOWN;
    }
    break;

  case LIFTER_DOWN:

    control_pos_lifter_front(1890, 200);
    control_pos_lifter_behind(1890, 245);

    if (abs(1890 - pos[6]) < 8 && abs(1890 - (-pos[7])) < 8)
    {
      if (millis() - grip_timer >= 500)
      {
        grip_timer = millis();
        sensor_ready = false;
        last_proxy = false;
        grip_step = WAIT_SENSOR_READY;

        std_msgs__msg__Bool bool_msg;
        bool_msg.data = true;
        RCSOFTCHECK(rcl_publish(&robot_up_publisher, &bool_msg, NULL));
      }
    }
    else
    {
      grip_timer = millis(); // reset timer kalau belum stabil
    }

    break;

  case WAIT_SENSOR_READY:

    // control_pos(-890, 100);

    control_pos_lifter_front(1890, 200);
    control_pos_lifter_behind(1890, 245);

    if (millis() - grip_timer >= 3000)
    {
      grip_timer = millis();
      grip_step = WAIT_AFTER_DOWN;
      sensor_ready = true;
      proxy_latched = true; // anggap sudah terdeteksi
    }

    break;

  case WAIT_AFTER_DOWN:

    if (millis() - grip_timer >= 1500)
    {
      grip_timer = millis();
      grip_step = GRIPPER_CLOSE;
    }
    break;

  case GRIPPER_CLOSE:

    srv.write(64);

    if (millis() - grip_timer >= 500)
    {
      grip_timer = millis();
      homed_lifter = false;
      grip_step = LIFTER_UP;
    }
    break;

  case LIFTER_UP:

    control_pos_lifter_front(2650, 200);
    control_pos_lifter_behind(2650, 245);

    if (abs(2650 - pos[6]) < 8 && abs(2650 - (-pos[7])) < 8)
    {
      if (millis() - grip_timer >= 500)
      {
        grip_timer = millis();
        grip_step = MOTOR_SLIDE;
      }
    }
    else
    {
      grip_timer = millis();
    }

    break;

  case MOTOR_SLIDE:

    static bool cmd_sent = false;
    if (!cmd_sent)
    {
      slide_done = false;
      sendSlideCommand(1); // kirim perintah ke LIMIT4
      cmd_sent = true;
    }
    if (slide_done)
    {
      cmd_sent = false;
      slide_done = false;
      grip_timer = millis();
      grip_step = WAIT_MOTOR_HOMING;
    }

    break;

  case WAIT_MOTOR_HOMING:

    if (millis() - grip_timer >= 1500)
    {
      grip_timer = millis();
      grip_step = SOL_HOLDER;
    }

    break;

  case SOL_HOLDER:

    // digitalWrite(solenoidHolder, HIGH);

    static bool sol_on_sent = false;
    if (!sol_on_sent)
    {
      sendSolenoidCommand(false); // suruh Bluepill: solenoidHolder HIGH
      sol_on_sent = true;
    }

    if (millis() - grip_timer > 500)
    {
      grip_timer = millis();
      sol_on_sent = false;
      grip_step = OPEN_SOL_HOLDER;
    }

    break;

  case OPEN_SOL_HOLDER:

    if (ir_button_a)
    {
      // digitalWrite(solenoidHolder, LOW);
      // ir_start = false;
      ir_button_a = false;
      sendSolenoidCommand(true);
      solenoid_triggered = true;
      grip_timer = millis();
    }

    if (solenoid_triggered && millis() - grip_timer >= 500)
    {
      solenoid_triggered = false;
      grip_step = WAIT_GRIPPER_OPEN;
    }

    break;

  case WAIT_GRIPPER_OPEN:

    if (millis() - grip_timer >= 1300)
    {
      grip_timer = millis();
      grip_step = GRIPPER_OPEN;
    }

    break;

  case GRIPPER_OPEN:
    srv.write(150);

    if (millis() - grip_timer >= 500)
    {
      grip_step = WAIT_GRIPPER_CLOSE;
    }

    break;

  case WAIT_GRIPPER_CLOSE:

    if (millis() - grip_timer >= 1400)
    {
      grip_timer = millis();
      grip_step = GRIPPER_CLOSE_2;
      homed = false;
    }

    break;

  case GRIPPER_CLOSE_2:

    srv.write(67);

    // if (!homed)
    // {
    //   limitMotor(0);
    // }

    static bool cmd_sent2 = false;
    if (!cmd_sent2)
    {
      slide_done = false;
      sendSlideCommand(0); // balik ke LIMIT3
      cmd_sent2 = true;
    }
    if (slide_done)
    {
      cmd_sent2 = false;
      slide_done = false;
      srv.write(150);
      grip_step = WAIT_MOTOR_HOMING2;
      grip_timer = millis();

      disable_tof_trigger = true;
      proxy_latched = false;
      lifter_triggered = false;
      grip_step = GRIP_IDLE;

      std_msgs__msg__Bool bool_msg;
      bool_msg.data = true;
      RCSOFTCHECK(rcl_publish(&lifter_down2_publisher, &bool_msg, NULL));

      // std_msgs__msg__Bool bool_msg;
      // bool_msg.data = true;
      // RCSOFTCHECK(rcl_publish(&lifter_down2_publisher, &bool_msg, NULL));
    }

    break;

  // case WAIT_MOTOR_HOMING2:

  //   control_pos_lifter_front(1890, 200);
  //   control_pos_lifter_behind(1890, 245);

  //   if (abs(1890 - pos[6]) < 8 && abs(1890 - (-pos[7])) < 8)
  //   {
  //     if (millis() - grip_timer >= 500)
  //     {
  //       grip_timer = millis();

  //       disable_tof_trigger = true;
  //       proxy_latched = false;
  //       lifter_triggered = false;
  //       grip_step = GRIP_IDLE;

  //       std_msgs__msg__Bool bool_msg;
  //       bool_msg.data = true;
  //       RCSOFTCHECK(rcl_publish(&lifter_down2_publisher, &bool_msg, NULL));
  //     }
  //   }
  //   else
  //   {
  //     grip_timer = millis();
  //   }
  //   break;

    //   case LIFTER_DOWN_2:

    //   control_pos(-890, 100);

    //   if (millis() - grip_timer >= 1000)
    //   {
    //     grip_timer = millis();
    //     disable_tof_trigger = true;
    //     proxy_latched = false;
    //     grip_step = GRIP_IDLE;
    //   }

    //   break;

  default:
    grip_step = GRIP_IDLE;
    break;
  }
}

struct timespec getTime()
{
  struct timespec tp = {0};

  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void flashLED(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }

  delay(1000);
}

void allbuttonCallback(const void *msgin)
{
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  allbutton_msg = *msg;
  switch (allbutton_msg.data)
  {
  case 0:
    button.A = 1;
    break;
  case 1:
    button.B = 1;
    break;
  case 2:
    button.X = 1;
    break;
  case 3:
    button.Y = 1;
    break;
  case 4:
    button.LB = 1;
    break;
  case 5:
    button.RB = 1;
    break;
  case 6:
    button.LT = 1;
    break;
  case 7:
    button.RT = 1;
    break;
  case 8:
    button.select = 1;
    break;
  case 9:
    button.start = 1;
    break;
  case 10:
    button.home = 1;
    break;

  default:
    button.A = 0;
    button.B = 0;
    button.X = 0;
    button.Y = 0;
    button.LB = 0;
    button.RB = 0;
    button.LT = 0;
    button.RT = 0;
    button.select = 0;
    button.start = 0;
    button.home = 0;
    break;
  }
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
  if (pwmVal > 0)
  {
    analogWrite(cwPin, fabs(pwmVal));
    analogWrite(ccwPin, 0);
  }
  else if (pwmVal < 0)
  {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, fabs(pwmVal));
  }
  else
  {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, 0);
  }
}

template <int i>
void readEncoder()
{
  int b = digitalRead(encb[i]);
  if (b > 0)
  {
    pos[i]++;
  }
  else
  {
    pos[i]--;
  }
}