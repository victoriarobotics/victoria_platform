/*
 * Victoria Teensy ROS Node
 * 
 * This code is the ROS node that executes on the Teensy processor
 * for the Victoria RoboMagellan team.
 * 
 * It uses the rosserial library to communicate with ROS. It uses
 * Arduino and Teensy libraries to interact with systems and
 * circuits connected to the Teensy processor. This code assumes
 * that a Teensy 3.5 is used.
 * 
 * https://github.com/victoriarobotics
 * 
 */

// Teensy includes
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>  // 
#include <i2c_t3.h>
#include <LSM6.h>
#include <LIS3MDL.h>

// ROS includes
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

// Teensy pin definitions
#define ENCODER_LEFT_PIN_1 29   // Teensy digital pin 29
#define ENCODER_LEFT_PIN_2 30   // Teensy digital pin 30
#define ENCODER_RIGHT_PIN_1 31  // Teensy digital pin 31
#define ENCODER_RIGHT_PIN_2 32  // Teensy digital pin 32
#define BUMPER_LEFT_PIN A15      // Teensy analog pin 15
#define BUMPER_RIGHT_PIN A14     // Teensy analog pin 14

// TRex motor controller
HardwareSerial trex = HardwareSerial();

#define BLINK_DURATION 2000
#define BLINK_PIN 13 // Teensy digital pin 13 for LED
int blink_state;
unsigned long next_blink_time;

char debug_str[80];

// Victoria constants
#define TRACK_RADIUS 0.235  // in meters
#define WHEEL_RADIUS 0.127  // in meters
#define MAX_SPEED 4.71      // in meters/second

// Bumper sensors
#define BUMPER_THRESHOLD 300  // Threshold that will indicate robot should stop
bool bumper_stop = false;
unsigned int bumper_left_base;
unsigned int bumper_right_base;

// Motor encoders
Encoder encoder_left(ENCODER_LEFT_PIN_1, ENCODER_LEFT_PIN_2);
Encoder encoder_right(ENCODER_RIGHT_PIN_1, ENCODER_RIGHT_PIN_2);
unsigned long encoder_left_pos;
unsigned long encoder_right_pos;
double motor_left_speed;
double motor_right_speed;

/*
double ang_v_l[10];
double ang_v_r[10];
int velocity_index = 0;
ros::Time last_encoder_read_time;
ros::Time next_encoder_read_time;
ros::Duration next_encoder_read_duration = ros::Duration(3e-3);
*/

// IMU and Magnetometer
LSM6 imu;
LIS3MDL mag;
bool imu_err = false;
bool mag_err = false;

// ROS node handle
ros::NodeHandle ros_nh;

ros::Time last_cmd_vel_time;
std_msgs::String ros_debug_msg;
ros::Publisher ros_bumper_pub("bumper_info", &ros_debug_msg);
ros::Publisher ros_encoder_pub("encoder_info", &ros_debug_msg);
ros::Publisher ros_imu_pub("imu_info", &ros_debug_msg);
ros::Publisher ros_magnetometer_pub("magnetometer_info", &ros_debug_msg);
ros::Publisher ros_debug_pub("v_debug_info", &ros_debug_msg);

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  snprintf(debug_str, sizeof(debug_str), "linear: %6f %6f %6f    angular: %6f %6f %6f",
    cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.linear.z,
    cmd_vel_msg.angular.x, cmd_vel_msg.angular.y, cmd_vel_msg.angular.z);
  ros_debug_msg.data = debug_str;
  ros_debug_pub.publish(&ros_debug_msg);
  
  last_cmd_vel_time = ros_nh.now();

  if (bumper_stop) {
    return;
  }

  // linear wheel velocity
  double vl = cmd_vel_msg.linear.x - (cmd_vel_msg.angular.z * TRACK_RADIUS);
  double vr = cmd_vel_msg.linear.x + (cmd_vel_msg.angular.z * TRACK_RADIUS);

  // angular velocity
  double wl = vl / WHEEL_RADIUS;
  double wr = vr / WHEEL_RADIUS;

  // Translate values to between -1 and 1
  motor_left_speed = max(-1, min(1, wl / MAX_SPEED));
  motor_right_speed = max(-1, min(1, wr / MAX_SPEED));

  snprintf(debug_str, sizeof(debug_str), "l: %6f %6f %6f    r: %6f %6f %6f",
    vl, wl, motor_left_speed,
    vr, wr, motor_right_speed);
  ros_debug_msg.data = debug_str;
  ros_debug_pub.publish(&ros_debug_msg);
}

// ROS cmd_vel subscriber
// Threshold, in nanoseconds, to wait for a cmd_vel message else take action
// to stop robot activity
#define CMD_VEL_TIMEOUT_THRESHOLD_NS 200000000
void cmdVelCallback(const geometry_msgs::Twist& twist_msg);
ros::Subscriber<geometry_msgs::Twist> ros_cmd_vel_sub("cmd_vel", cmdVelCallback);

// ROS Odometry publisher
nav_msgs::Odometry ros_odom_msg;
ros::Publisher ros_odom_pub("odom", &ros_odom_msg);

// ROS Odometry broadcaster
geometry_msgs::TransformStamped ros_odom_transform;
tf::TransformBroadcaster ros_odom_broadcaster;

ros::Time current_time;
ros::Time last_time;

char ros_odom_header_frame_id[] = "/odom";
char ros_odom_child_frame_id[] = "/base_link";

//Pose pose;
  
void setup() {
  // Setup basic I2C master mode pins 18/19, external pullups, 400kHz, 200ms default timeout
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(200000); // 200ms

  // Setup IMU and magenetometer that use I2C
  if (imu.init()) {
    imu.enableDefault();
    // TODO: set timeout?
  } else {
    imu_err = true;
  }
  
  if (mag.init()) {
    mag.enableDefault();
    // TODO: set timeout?
  } else {
    mag_err = true;
  }
    
  // Start serial port to TRex
  trex.begin(19200);

  bumper_left_base = 5000; //analogRead(BUMPER_LEFT_PIN);
  bumper_right_base = 5000; //analogRead(BUMPER_RIGHT_PIN);

  next_blink_time = millis();
  blink_state = LOW;
  pinMode(BLINK_PIN, OUTPUT);

  // Initialize ros node
  ros_nh.initNode();

  // Initialize broadcasters
  ros_odom_transform.header.frame_id = ros_odom_header_frame_id;
  ros_odom_transform.child_frame_id = ros_odom_child_frame_id;
  ros_odom_broadcaster.init(ros_nh);

  // Initialize ros publishers
  ros_odom_msg.header.frame_id = ros_odom_header_frame_id;
  ros_odom_msg.child_frame_id = ros_odom_child_frame_id;
  ros_nh.advertise(ros_odom_pub);
  ros_nh.advertise(ros_bumper_pub);
  ros_nh.advertise(ros_encoder_pub);
  ros_nh.advertise(ros_imu_pub);
  ros_nh.advertise(ros_magnetometer_pub);
  ros_nh.advertise(ros_debug_pub);

  // Initialize ros subscribers
  ros_nh.subscribe(ros_cmd_vel_sub);
  
  current_time = ros_nh.now();
  last_time = current_time;
  last_cmd_vel_time = current_time;
  //last_encoder_read_time = current_time;
  //next_encoder_read_time = current_time + next_encoder_read_duration;
  encoder_left_pos = 0.0;
  encoder_right_pos = 0.0;
  motor_left_speed = 0.0;
  motor_right_speed = 0.0;
}

void loop() {
  
    
  current_time = ros_nh.now();

  // If we have not received any cmd_vel messages for a while,
  // something is wrong, stop the robot.
  if (current_time.toNsec() - last_cmd_vel_time.toNsec() 
        > CMD_VEL_TIMEOUT_THRESHOLD_NS) {
    // TODO: Do something to stop the robot!
  }
  
  // Read bumper sensors
  unsigned int new_bumper_left = analogRead(BUMPER_LEFT_PIN);
  unsigned int new_bumper_right = analogRead(BUMPER_RIGHT_PIN);

  // Check bumper thresholds to stop robot
  if (new_bumper_left < 450 || new_bumper_right < 450) {
    bumper_stop = true;
    motor_left_speed = 0;
    motor_right_speed = 0;
  } else {
    bumper_stop = false;
  }

  /*
  // Read encoders
  if (current_time > next_encoder_read_time) {
    unsigned long new_encoder_l_pos = encoder_left.read();
    unsigned long new_encoder_r_pos = encoder_right.read();

    // TODO handle overflow/rest to 0
    unsigned long diff_l = new_encoder_l_pos - encoder_left_pos;
    unsigned long diff_r = new_encoder_r_pos - encoder_right_pos;
    double diff_t = current_time - last_encoder_read_time;

    ang_vel_l[velocity_index] = (diff_l/TICKS_PER_RADIAN)/diff_t;
    ang_vel_r[velocity_index] = (diff_r/TICKS_PER_RADIAN)/diff_t;
    
    velocity_index = (velocity_index+1)%10;
    encoder_left_pos = new_encoder_left_pos;
    encoder_right_pos = new_encoder_right_pos;
    last_encoder_read_time = current_time;
    next_encoder_read_time = current_time + next_encoder_read_duration;
  }
  unsigned long new_encoder_left_pos = encoder_left.read();
  unsigned long new_encoder_right_pos = encoder_right.read();


  double dt = (current_time - last_time).toSec();
  */
    
  // Read the bumper sensors
  snprintf(debug_str, sizeof(debug_str), "L: %d    R: %d",
    analogRead(BUMPER_LEFT_PIN), analogRead(BUMPER_RIGHT_PIN));
  ros_debug_msg.data = debug_str;
  ros_bumper_pub.publish(&ros_debug_msg);
  
  // Read the encoders
  snprintf(debug_str, sizeof(debug_str), "L: %ld    R: %ld",
    encoder_left.read(), encoder_right.read());
  ros_debug_msg.data = debug_str;
  ros_encoder_pub.publish(&ros_debug_msg);
  
  // Read the IMU
  if (imu_err) {
    ros_debug_msg.data = "IMU failure!";
  } else {
    long i2c_start_time = millis();
    imu.read();
    long i2c_duration = millis() - i2c_start_time;
  
    snprintf(debug_str, sizeof(debug_str), "A: %6d %6d %6d    G: %6d %6d %6d   I2C Duration: %ld",
      imu.a.x, imu.a.y, imu.a.z,
      imu.g.x, imu.g.y, imu.g.z,
      i2c_duration);
    ros_debug_msg.data = debug_str;
  }
  ros_imu_pub.publish(&ros_debug_msg);

  // Read the Magnetometer
  if (mag_err) {
    ros_debug_msg.data = "Magnetometer failure!";
  } else {
    long i2c_start_time = millis();
    mag.read();
    long i2c_duration = millis() - i2c_start_time;
  
    snprintf(debug_str, sizeof(debug_str), "M: %6d %6d %6d   I2C Duration: %ld",
      mag.m.x, mag.m.y, mag.m.z,
      i2c_duration);
    ros_debug_msg.data = debug_str;
  }
  ros_magnetometer_pub.publish(&ros_debug_msg);

  // Broadcast odometry transform
  ros_odom_transform.header.stamp = current_time;

  // TODO: set all the transform data

  //ros_odom_broadcaster.sendTransform(ros_odom_transform);
  
  // Publish Odometry
  ros_odom_msg.header.stamp = current_time;
  
  // TODO: Set all the odometry data
  
  //ros_odom_pub.publish(&ros_odom_msg);
  
  last_time = current_time;
  
  ros_nh.spinOnce();

  setMotors(motor_left_speed, motor_right_speed);
  
  // Indicate to outside world that something is going on
  if (millis() > next_blink_time) {
    if (blink_state == LOW) {
      blink_state = HIGH;
    } else {
      blink_state = LOW;
    }
    digitalWrite(BLINK_PIN, blink_state);
    next_blink_time = millis() + BLINK_DURATION;
  }
}

/*
 * Sets the speed for the motors. Values are expected
 * to be between -1 and 1. Values outside this range
 * will be pinned.
 */
void setMotors(double motor_left_speed, double motor_right_speed) {
  byte command_byte_left = 0xC4;
  if (motor_left_speed < 0) {
    command_byte_left = command_byte_left | 0x01;
  } else {
    command_byte_left = command_byte_left | 0x02;
  }
  int trex_left_speed = min(127, 127.0 * abs(motor_left_speed));

  byte command_byte_right = 0xCC;
  if (motor_right_speed < 0) {
    command_byte_right = command_byte_right | 0x02;
  } else {
    command_byte_right = command_byte_right | 0x01;
  }
  int trex_right_speed = min(127, 127.0 * abs(motor_right_speed));
  
  snprintf(debug_str, sizeof(debug_str), "cmd_l: %6f %6d %6d    cmd_r: %6f %6d %6d",
    motor_left_speed, command_byte_left, trex_left_speed,
    motor_right_speed, command_byte_right, trex_right_speed);
  ros_debug_msg.data = debug_str;
  ros_debug_pub.publish(&ros_debug_msg);
  
  trex.write(command_byte_left);
  trex.write((byte)trex_left_speed);
  trex.write(command_byte_right);
  trex.write((byte)trex_right_speed);
}

