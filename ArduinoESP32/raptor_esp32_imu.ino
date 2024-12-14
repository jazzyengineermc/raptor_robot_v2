#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <sensor_msgs/msg/imu.h>
Adafruit_MPU6050 mpu;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;

std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.13, wheel_sep = 0.51;

// Encoder Pins
#define LeftEncoder_C1 18
#define LeftEncoder_C2 19
#define RightEncoder_C1 17
#define RightEncoder_C2 16

// Pin Definitions
#define IN1_PIN 32    //left forward
#define IN2_PIN 33    //left backward
#define IN3_PIN 27    //right forward
#define IN4_PIN 14    //right backward

// Motor control variables
int motorSpeedLeft = 0;
int motorSpeedRight = 0;

// Encoder variables
int LeftEncoderCount = 0;
int RightEncoderCount = 0;

void LeftEncoderCallback();
void RightEncoderCallback();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function prototypes
void setMotorSpeed(int speedLeft, int speedRight);

void error_loop(){
  while(1){
    delay(100);
  }
}

int limitToMaxValue(int value, int maxLimit) {
  if (value > maxLimit) {
    return maxLimit;
  } else {
    return value;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    right_encoder_msg.data = RightEncoderCount;
    left_encoder_msg.data = LeftEncoderCount;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    imu_msg.header.frame_id.data = (char*)"imu_frame";
    imu_msg.header.frame_id.size = 9;
    imu_msg.header.stamp.sec = (int64_t)tv.tv_sec;
    imu_msg.header.stamp.nanosec = (int64_t)tv.tv_usec;
    
    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.angular_velocity.x = a.gyro.x;
    imu_msg.angular_velocity.y = a.gyro.y;
    imu_msg.angular_velocity.z = a.gyro.z;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));

  }
}

// Twist message callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Calculate motor speeds based on twist message
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Calculate individual motor speeds
  motorSpeedLeft = (linear/wheel_rad) - (angular*wheel_sep)/(2.0*wheel_rad)*10;
  motorSpeedRight = (linear/wheel_rad) + (angular*wheel_sep)/(2.0*wheel_rad)*10;

  if (motorSpeedLeft > 0) {
  motorSpeedLeft = motorSpeedLeft +40;
  } 
  if (motorSpeedLeft < 0) {
  motorSpeedLeft = motorSpeedLeft - 40;
  }
  
  if (motorSpeedRight > 0) {
  motorSpeedRight = motorSpeedRight +40;
  } 
  if (motorSpeedRight < 0) {
  motorSpeedRight = motorSpeedRight - 40;
  }
  // Set motor speeds
  setMotorSpeed(motorSpeedLeft, motorSpeedRight);
}

void setMotorSpeed(int speedLeft, int speedRight) {

if (speedLeft > 0) {

  digitalWrite(IN2_PIN, LOW);
  analogWrite(IN1_PIN, abs(limitToMaxValue(speedLeft, 250)));
} else {
  digitalWrite(IN1_PIN, LOW);
  analogWrite(IN2_PIN, abs(limitToMaxValue(speedLeft, 250)));
}

  // Set right motor direction and speed
  if (speedRight > 0) {
    digitalWrite(IN4_PIN, LOW);
    analogWrite(IN3_PIN, abs(limitToMaxValue(speedRight, 250)));
  } else {
    digitalWrite(IN3_PIN, LOW);
    analogWrite(IN4_PIN, abs(limitToMaxValue(speedRight, 250)));
  }

  if (speedLeft == 0 && speedRight == 0  ) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN1_PIN, abs(limitToMaxValue(speedLeft, 250)));
    analogWrite(IN3_PIN, abs(limitToMaxValue(speedLeft, 250)));
  }
  
}

void setup() {
  set_microros_transports();
  mpu.begin();
  
  pinMode(LeftEncoder_C1, INPUT_PULLUP);
  pinMode(LeftEncoder_C2, INPUT_PULLUP);
  pinMode(RightEncoder_C1, INPUT_PULLUP);
  pinMode(RightEncoder_C2, INPUT_PULLUP);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LeftEncoder_C1), LeftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(RightEncoder_C1), RightEncoderCallback, RISING);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  delay(2000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "raptor_esp32", "", &support));

  // Create twist subscriber
  RCCHECK(rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lwheel"));

  RCCHECK(rclc_publisher_init_default(
    &right_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "rwheel"));

  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));
    
  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
    
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(100);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void LeftEncoderCallback() {
  if (digitalRead(LeftEncoder_C1) == digitalRead(LeftEncoder_C2)) {
    LeftEncoderCount++;
  } else {
    LeftEncoderCount--;
  }
}

void RightEncoderCallback() {
  if (digitalRead(RightEncoder_C1) == digitalRead(RightEncoder_C2)) {
    RightEncoderCount++;
  } else {
    RightEncoderCount--;
  }
}