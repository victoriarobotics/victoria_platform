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
#include <Encoder.h>
#include <i2c_t3.h>
#include <LSM6.h>
#include <LIS3MDL.h>

// Other includes
#include <Timer.h>

// ROS includes
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

// Teensy pin definitions
#define BLINK_PIN           13  // Teensy digital pin 13
#define ENCODER_LEFT_PIN_1  29  // Teensy digital pin 29
#define ENCODER_LEFT_PIN_2  30  // Teensy digital pin 30
#define ENCODER_RIGHT_PIN_1 31  // Teensy digital pin 31
#define ENCODER_RIGHT_PIN_2 32  // Teensy digital pin 32
#define BUMPER_LEFT_PIN     A15 // Teensy analog pin 15
#define BUMPER_RIGHT_PIN    A14 // Teensy analog pin 14

// Timer
Timer timer;

// TRex motor controller
HardwareSerial trex = HardwareSerial();
double motor_left_speed;
double motor_right_speed;

// Victoria constants
#define TICKS_PER_RADIAN 9072 // TODO(mwomack): Verify real value
#define TRACK_RADIUS 0.235  // in meters
#define WHEEL_RADIUS 0.127  // in meters
#define MAX_SPEED 4.71      // in meters/second

// Bumper sensors
bool bumper_stop = false; // Set to true if bumpers indicate robot should stop
unsigned int bumper_left;
unsigned int bumper_right;

// Motor encoders
Encoder encoder_left(ENCODER_LEFT_PIN_1, ENCODER_LEFT_PIN_2);
Encoder encoder_right(ENCODER_RIGHT_PIN_1, ENCODER_RIGHT_PIN_2);
unsigned long encoder_left_pos;
unsigned long encoder_right_pos;
ros::Time last_encoder_read_time;

#define NUM_VEL_SAMPLES 10
double ang_v_samples_l[NUM_VEL_SAMPLES];
double ang_v_samples_r[NUM_VEL_SAMPLES];
int velocity_index = 0;

// IMU and Magnetometer
LSM6 imu;
LIS3MDL mag;
bool imu_err = false;
bool mag_err = false;

// ROS node handle
ros::NodeHandle ros_nh;

// ROS cmd_vel subscriber
// Threshold, in nanoseconds, to wait for a cmd_vel message else take action
// to stop robot activity
#define CMD_VEL_TIMEOUT_THRESHOLD 0.2 // 200 milliseconds
ros::Time last_cmd_vel_time;
bool cmd_vel_timeout_stop = false; // Set to true if time out threshold has been exceeded
void cmdVelCallback(const geometry_msgs::Twist& twist_msg);
ros::Subscriber<geometry_msgs::Twist> ros_cmd_vel_sub("cmd_vel", cmdVelCallback);

// ROS Odometry publisher
nav_msgs::Odometry ros_odom_msg;
ros::Publisher ros_odom_pub("odom", &ros_odom_msg);
ros::Time last_odom_publish_time;

// ROS Odometry broadcaster
geometry_msgs::TransformStamped ros_odom_transform;
tf::TransformBroadcaster ros_odom_broadcaster;

// Position variables
double x;
double y;
double th;

ros::Time current_time;
ros::Time previous_time;

char ros_odom_header_frame_id[] = "/odom";
char ros_odom_child_frame_id[] = "/base_link";

// ROS Debug publishers
#define ENABLE_BUMPER_DEBUG  // Uncomment to publish bumper debug info
#define ENABLE_ENCODER_DEBUG  // Uncomment to publish encoder debug info
#define ENABLE_IMU_DEBUG  // Uncomment to publish imu debug info
#define ENABLE_MAGNETOMETER_DEBUG  // Uncomment to publish magnetometer debug info
#define ENABLE_TEENSY_DEBUG  // Uncomment to publish teensy debug info

char debug_str[80];
std_msgs::String ros_debug_msg;

#ifdef ENABLE_BUMPER_DEBUG
ros::Publisher ros_bumper_debug_pub("bumper_debug", &ros_debug_msg);
#endif

#ifdef ENABLE_ENCODER_DEBUG
ros::Publisher ros_encoder_debug_pub("encoder_debug", &ros_debug_msg);
#endif

#ifdef ENABLE_IMU_DEBUG
ros::Publisher ros_imu_debug_pub("imu_debug", &ros_debug_msg);
#endif

#ifdef ENABLE_MAGNETOMETER_DEBUG
ros::Publisher ros_magnetometer_debug_pub("magnetometer_debug", &ros_debug_msg);
#endif

#ifdef ENABLE_TEENSY_DEBUG
ros::Publisher ros_teensy_debug_pub("teensy_debug_info", &ros_debug_msg);
#endif

// Debug blink
#define BLINK_DURATION 2000
int blink_state = LOW;
  
void setup() {
  // Setup basic I2C master mode pins 18/19, external pullups, 400kHz, 200ms default timeout
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(200000); // 200ms

  // Setup IMU and magenetometer that use I2C
  if (imu.init()) {
    imu.enableDefault();
    // TODO(mwomack): set timeout?
  } else {
    imu_err = true;
  }
  
  if (mag.init()) {
    mag.enableDefault();
    // TODO(mwomack): set timeout?
  } else {
    mag_err = true;
  }
    
  // Start serial port to TRex
  trex.begin(19200);

  // Setup pin configurations
  pinMode(BLINK_PIN, OUTPUT);

  encoder_left_pos = 0.0;
  encoder_right_pos = 0.0;
  motor_left_speed = 0.0;
  motor_right_speed = 0.0;
  x = 0.0;
  y = 0.0;
  th = 0.0;

  // Initialize and connect to ros
  ros_nh.initNode();
  while(!ros_nh.connected()) {
    ros_nh.spinOnce();
  }

  // Get ros parameters
  int read_encoders_freq_hz;
  if (!ros_nh.getParam("read_encoders_freq_hz", &read_encoders_freq_hz)) { 
    read_encoders_freq_hz = 300;
  }
  
  int publish_odom_freq_hz;
  if (!ros_nh.getParam("publish_odom_freq_hz", &publish_odom_freq_hz)) { 
    publish_odom_freq_hz = 100;
  }
  
  int publish_imu_freq_hz;
  if (!ros_nh.getParam("publish_imu_freq_hz", &publish_imu_freq_hz)) { 
    publish_imu_freq_hz = 2;
  }
  
  int publish_magnetometer_freq_hz;
  if (!ros_nh.getParam("publish_magnetometer_freq_hz", &publish_magnetometer_freq_hz)) { 
    publish_magnetometer_freq_hz = 2;
  }
  
  // Initialize broadcasters
  ros_odom_transform.header.frame_id = ros_odom_header_frame_id;
  ros_odom_transform.child_frame_id = ros_odom_child_frame_id;
  ros_odom_broadcaster.init(ros_nh);

  // Initialize ros publishers
  ros_odom_msg.header.frame_id = ros_odom_header_frame_id;
  ros_odom_msg.child_frame_id = ros_odom_child_frame_id;
  ros_nh.advertise(ros_odom_pub);

  // Initialize ros subscribers
  ros_nh.subscribe(ros_cmd_vel_sub);

  // Setup Timer callbacks
  timer.every((1.0/read_encoders_freq_hz) * 1000, doReadEncoders);
  timer.every((1.0/publish_odom_freq_hz) * 1000, doPublishOdom);
  timer.every((1.0/publish_imu_freq_hz) * 1000, doPublishImu);
  timer.every((1.0/publish_magnetometer_freq_hz) * 1000, doPublishMagnetometer);
  timer.every(BLINK_DURATION, doBlink);
  
  current_time = ros_nh.now();
  previous_time = current_time;
  last_cmd_vel_time = current_time;
  last_odom_publish_time = current_time;
  last_encoder_read_time = current_time;

  // Initialize ros debug publishers
#ifdef ENABLE_BUMPER_DEBUG
  ros_nh.advertise(ros_bumper_debug_pub);
  timer.every(500, doBumperDebug);
#endif
#ifdef ENABLE_ENCODER_DEBUG
  ros_nh.advertise(ros_encoder_debug_pub);
  timer.every(500, doEncoderDebug);
#endif
#ifdef ENABLE_IMU_DEBUG
  ros_nh.advertise(ros_imu_debug_pub);
  timer.every(500, doImuDebug);
#endif
#ifdef ENABLE_MAGNETOMETER_DEBUG
  ros_nh.advertise(ros_magnetometer_debug_pub);
  timer.every(500, doMagnetometerDebug);
#endif
#ifdef ENABLE_TEENSY_DEBUG
  ros_nh.advertise(ros_teensy_debug_pub);
#endif
}

void loop() {
  // Check bumper thresholds to stop robot
  bumper_left = analogRead(BUMPER_LEFT_PIN);
  bumper_right = analogRead(BUMPER_RIGHT_PIN);
  bumper_stop = bumper_left < 450 || bumper_right < 450;
  if (bumper_stop) {
    motor_left_speed = 0;
    motor_right_speed = 0;
  }

  // Set current time for all following code
  current_time = ros_nh.now();

  // Update any timer callbacks
  timer.update();

  // Update ROS
  ros_nh.spinOnce();
  
  // If have not received cmd_vel messages in threshold time,
  // something is wrong, stop the robot.
  cmd_vel_timeout_stop = (current_time.toSec() - last_cmd_vel_time.toSec()) > CMD_VEL_TIMEOUT_THRESHOLD;
  if (cmd_vel_timeout_stop) {
    motor_left_speed = 0;
    motor_right_speed = 0;
  }

  // Set the motors to the current speed
  setMotors(motor_left_speed, motor_right_speed);

  // Current time is now the previous time
  previous_time = current_time;
}

/*
 * Callback used by ros_cmd_vel_sub to handle cmd_vel commands.
 */
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  last_cmd_vel_time = ros_nh.now();

  if (bumper_stop) {
    // TODO(mwomack): Allow for reverse when bumper_stop
    return;
  }
  
  // Calculate wheel linear velocity
  double vl = cmd_vel_msg.linear.x - (cmd_vel_msg.angular.z * TRACK_RADIUS);
  double vr = cmd_vel_msg.linear.x + (cmd_vel_msg.angular.z * TRACK_RADIUS);

  // Calculate wheel angular velocity
  double wl = vl / WHEEL_RADIUS;
  double wr = vr / WHEEL_RADIUS;

  // Calculate motor speed between -1 and 1
  motor_left_speed = max(-1, min(1, wl / MAX_SPEED));
  motor_right_speed = max(-1, min(1, wr / MAX_SPEED));
}

/*
 * Reads the current values of the motor encoders and uses
 * the difference with the previous value from the encoders
 * to calculate the angular velocity of each wheel.
 */
void doReadEncoders() {
  ros::Time current_time = ros_nh.now();
  unsigned long new_encoder_left_pos = encoder_left.read();
  unsigned long new_encoder_right_pos = encoder_right.read();

  // TODO(mwomack): handle overflow/reset to 0
  unsigned long diff_l = new_encoder_left_pos - encoder_left_pos;
  unsigned long diff_r = new_encoder_right_pos - encoder_right_pos;
  double diff_t = current_time.toSec() - last_encoder_read_time.toSec();

  ang_v_samples_l[velocity_index] = (diff_l/TICKS_PER_RADIAN)/diff_t;
  ang_v_samples_r[velocity_index] = (diff_r/TICKS_PER_RADIAN)/diff_t;

  velocity_index = (velocity_index + 1) % NUM_VEL_SAMPLES;
  encoder_left_pos = new_encoder_left_pos;
  encoder_right_pos = new_encoder_right_pos;
  last_encoder_read_time = current_time;
}

/*
 * Publish odometry to ros.
 */
void doPublishOdom() {
  ros::Time current_time = ros_nh.now();
  
  // Get the current wheel angular velocity
  double wl = getAngularVelocityFromSamples(ang_v_samples_l);
  double wr = getAngularVelocityFromSamples(ang_v_samples_r);
  
  // Convert to wheel linear velocity
  double vl = wl * WHEEL_RADIUS;
  double vr = wr * WHEEL_RADIUS;
  
  // Calculate robot vx and vth
  double vx = (vl + vr)/2;
  double vth = (vr - vl)/(2 * TRACK_RADIUS);
  
  // Calculate the deltas since
  double dt = current_time.toSec() - last_odom_publish_time.toSec();
  double delta_x = (vx * cos(th)) * dt;
  double delta_y = (vx * sin(th)) * dt;
  double delta_th = vth * dt;

  // Update position
  x += delta_x;
  y += delta_y;
  th += delta_th;
    
  // Broadcast odometry transform

  // Since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
  
  // First, we'll publish the transform over tf
  ros_odom_transform.header.stamp = current_time;
  ros_odom_transform.transform.translation.x = x;
  ros_odom_transform.transform.translation.y = y;
  ros_odom_transform.transform.translation.z = 0.0;
  ros_odom_transform.transform.rotation = odom_quat;
  ros_odom_broadcaster.sendTransform(ros_odom_transform);
  
  // Next, we'll publish the odometry message over ROS
  ros_odom_msg.header.stamp = current_time;

  //set the position
  ros_odom_msg.pose.pose.position.x = x;
  ros_odom_msg.pose.pose.position.y = y;
  ros_odom_msg.pose.pose.position.z = 0.0;
  ros_odom_msg.pose.pose.orientation = odom_quat;

  //set the velocity
  ros_odom_msg.twist.twist.linear.x = vx;
  ros_odom_msg.twist.twist.linear.y = 0.0;
  ros_odom_msg.twist.twist.angular.z = vth;
  
  ros_odom_pub.publish(&ros_odom_msg);
  
  // Remember current time for next publish
  last_odom_publish_time = current_time;
}

/*
 * Publish imu data to ros.
 */
void doPublishImu() {
  // TODO(mwomack)
}

/*
 * Publish magnetometer data to ros.
 */
void doPublishMagnetometer() {
  // TODO(mwomack)
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
  
  trex.write(command_byte_left);
  trex.write((byte)trex_left_speed);
  trex.write(command_byte_right);
  trex.write((byte)trex_right_speed);
}

/*
 * Returns the calculated angular velocity from all of
 * the samples. currently it just returns the average
 * of all the samples.
 */
double getAngularVelocityFromSamples(double* samples) {
  double total = 0;
  for (int count = 0; count < NUM_VEL_SAMPLES; count++) {
    total += samples[count];
  }
  return total/10.0;
}

// Callback for blink
void doBlink() {
  if (blink_state == LOW) {
    blink_state = HIGH;
  } else {
    blink_state = LOW;
  }
  digitalWrite(BLINK_PIN, blink_state);
}

#ifdef ENABLE_BUMPER_DEBUG
void doBumperDebug() {
  // Read the bumper sensors
  snprintf(debug_str, sizeof(debug_str), 
    "L: %d    R: %d    STOP: %d", bumper_left, bumper_right, bumper_stop);
  ros_debug_msg.data = debug_str;
  ros_bumper_debug_pub.publish(&ros_debug_msg);
}
#endif

#ifdef ENABLE_ENCODER_DEBUG
void doEncoderDebug() {
  // Read the encoders
  snprintf(debug_str, sizeof(debug_str), "L: %ld    R: %ld",
    encoder_left.read(), encoder_right.read());
  ros_debug_msg.data = debug_str;
  ros_encoder_debug_pub.publish(&ros_debug_msg);
}
#endif

#ifdef ENABLE_IMU_DEBUG
void doImuDebug() {
  // Read the IMU
  if (imu_err) {
    ros_debug_msg.data = "IMU failure!";
  } else {
    imu.read();
  
    snprintf(debug_str, sizeof(debug_str), 
      "A: %6d %6d %6d    G: %6d %6d %6d",
      imu.a.x, imu.a.y, imu.a.z,
      imu.g.x, imu.g.y, imu.g.z);
    ros_debug_msg.data = debug_str;
  }
  ros_imu_debug_pub.publish(&ros_debug_msg);
}
#endif

#ifdef ENABLE_MAGNETOMETER_DEBUG
void doMagnetometerDebug() {
  // Read the Magnetometer
  if (mag_err) {
    ros_debug_msg.data = "Magnetometer failure!";
  } else {
    mag.read();
  
    snprintf(debug_str, sizeof(debug_str),
      "M: %6d %6d %6d",
      mag.m.x, mag.m.y, mag.m.z);
    ros_debug_msg.data = debug_str;
  }
  ros_magnetometer_debug_pub.publish(&ros_debug_msg);
}
#endif
