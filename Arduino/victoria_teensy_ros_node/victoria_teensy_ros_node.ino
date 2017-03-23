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
/**
 * Macro to tell the Teensy Encoder library to use interrupts for best
 * performance.
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <i2c_t3.h>
#include <LSM6.h>
#include <LIS3MDL.h>

// Other includes
#include <Timer.h>

// ROS includes
#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <victoria_nav_msgs/Odom2DRaw.h>
#include <victoria_sensor_msgs/IMURaw.h>

// Teensy pin definitions
const int BLINK_PIN(13);           // Teensy digital pin 13
const int ENCODER_LEFT_PIN_1(29);  // Teensy digital pin 29
const int ENCODER_LEFT_PIN_2(30);  // Teensy digital pin 30
const int ENCODER_RIGHT_PIN_1(31); // Teensy digital pin 31
const int ENCODER_RIGHT_PIN_2(32); // Teensy digital pin 32
const int BUMPER_LEFT_PIN(A15);    // Teensy analog pin 15
const int BUMPER_RIGHT_PIN(A14);   // Teensy analog pin 14

// Timer
Timer timer;

// TRex motor controller
HardwareSerial trex = HardwareSerial();
double motor_left_speed;
double motor_right_speed;

// Victoria constants
const unsigned long TICKS_PER_RADIAN(9072); // TODO(mwomack): Verify real value
const double TRACK_RADIUS(0.235);  // in meters
const double WHEEL_RADIUS(0.127);  // in meters
const double MAX_SPEED(4.71);      // in meters/second

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

const int NUM_VEL_SAMPLES(10);
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
// Threshold, in seconds, to wait for a cmd_vel message else take action
// to stop robot activity
const double CMD_VEL_TIMEOUT_THRESHOLD(0.2); // 200 milliseconds
ros::Time last_cmd_vel_time;
bool cmd_vel_timeout_stop = false; // Set to true if time out threshold has been exceeded
void cmdVelCallback(const geometry_msgs::Twist& twist_msg);
ros::Subscriber<geometry_msgs::Twist> ros_cmd_vel_sub("cmd_vel", cmdVelCallback);

// Raw 2D Odometry publisher
victoria_nav_msgs::Odom2DRaw ros_raw_odom_msg;
ros::Publisher ros_raw_odom_pub("odom_2d_raw", &ros_raw_odom_msg);

// Raw IMU publisher
victoria_sensor_msgs::IMURaw ros_raw_imu_msg;
ros::Publisher ros_raw_imu_pub("imu_raw", &ros_raw_imu_msg);
ros::Time last_raw_odom_publish_time;

// Position of robot
geometry_msgs::Pose2D pose;

char ros_odom_header_frame_id[] = "/odom";
char ros_odom_child_frame_id[] = "/base_link";

char debug_str[80];
std_msgs::String ros_debug_msg;

ros::Publisher ros_bumper_debug_pub("bumper_debug", &ros_debug_msg);
ros::Publisher ros_encoder_debug_pub("encoder_debug", &ros_debug_msg);

// Debug blink
// Blink duration is in milliseconds.
const int BLINK_DURATION(500);
int blink_state = LOW;
  
void setup() {
  // Setup basic I2C master mode pins 18/19, external pullups, 400kHz, 200ms default timeout
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(200000); // 200ms

  // Setup IMU and magnetometer that use I2C
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
  pose.x = 0.0;
  pose.y = 0.0;
  pose.theta = 0.0;

  unsigned long rosStartTimeout = millis() + 500;

  // Initialize and connect to ros
  ros_nh.initNode();
  while(!ros_nh.connected() && millis() < rosStartTimeout) {
    ros_nh.spinOnce();
  }

  // Get ros parameters
  int read_encoders_freq_hz;
  if (!ros_nh.getParam("read_encoders_freq_hz", &read_encoders_freq_hz)) { 
    read_encoders_freq_hz = 300;
  }
  
  int publish_raw_odom_freq_hz;
  if (!ros_nh.getParam("publish_raw_odom_freq_hz", &publish_raw_odom_freq_hz)) { 
    publish_raw_odom_freq_hz = 100;
  }
  
  int publish_raw_imu_freq_hz;
  if (!ros_nh.getParam("publish_imu_freq_hz", &publish_raw_imu_freq_hz)) { 
    publish_raw_imu_freq_hz = 2;
  }

  int publish_bumper_debug_info_freq_hz;
  if (!ros_nh.getParam("publish_bumper_debug_info_freq_hz", &publish_bumper_debug_info_freq_hz)) { 
    publish_bumper_debug_info_freq_hz = 10;
  }

  int publish_encoder_debug_info_freq_hz;
  if (!ros_nh.getParam("publish_encoder_debug_info_freq_hz", &publish_encoder_debug_info_freq_hz)) { 
    publish_encoder_debug_info_freq_hz = 10;
  }

  // Initialize ros publishers
  ros_raw_odom_msg.header.frame_id = ros_odom_header_frame_id;
  ros_raw_odom_msg.child_frame_id = ros_odom_child_frame_id;
  ros_nh.advertise(ros_raw_odom_pub);
  ros_nh.advertise(ros_raw_imu_pub);

  // Initialize ros subscribers
  ros_nh.subscribe(ros_cmd_vel_sub);

  // Setup Timer callbacks
  timer.every(convertFreqToMillis(read_encoders_freq_hz), doReadEncoders);
  timer.every(convertFreqToMillis(publish_raw_odom_freq_hz), doPublishRawOdom);
  timer.every(convertFreqToMillis(publish_raw_imu_freq_hz), doPublishRawImu);
  timer.every(BLINK_DURATION, doBlink);

  ros::Time current_time = ros_nh.now();
  last_cmd_vel_time = current_time;
  last_raw_odom_publish_time = current_time;
  last_encoder_read_time = current_time;

  // Initialize ros debug publishers
  ros_nh.advertise(ros_bumper_debug_pub);
  ros_nh.advertise(ros_encoder_debug_pub);
  timer.every(convertFreqToMillis(publish_bumper_debug_info_freq_hz), doBumperDebug);
  timer.every(convertFreqToMillis(publish_encoder_debug_info_freq_hz), doEncoderDebug);
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

  ros::Time current_time = ros_nh.now();

  // Update any timer callbacks
  timer.update();

  // Update ROS
  ros_nh.spinOnce();
  
  // If no cmd_vel messages withing threshold time, something is wrong, stop the robot.
  cmd_vel_timeout_stop = (current_time.toSec() - last_cmd_vel_time.toSec()) > CMD_VEL_TIMEOUT_THRESHOLD;
  if (cmd_vel_timeout_stop) {
    motor_left_speed = 0;
    motor_right_speed = 0;
  }

  // Set the motors to the current speed
  setMotors(motor_left_speed, motor_right_speed);
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
 * Publish raw odometry to ros.
 */
void doPublishRawOdom() {
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
  double dt = current_time.toSec() - last_raw_odom_publish_time.toSec();
  double delta_x = (vx * cos(pose.theta)) * dt;
  double delta_y = (vx * sin(pose.theta)) * dt;
  double delta_th = vth * dt;

  // Update position
  pose.x += delta_x;
  pose.y += delta_y;
  pose.theta = normalize_angle(pose.theta + delta_th);
  
  // Publish the odometry message over ROS
  ros_raw_odom_msg.header.stamp = current_time;

  // set the position
  ros_raw_odom_msg.pose = pose;

  // set the velocity
  ros_raw_odom_msg.twist.vx = vx;
  ros_raw_odom_msg.twist.vy = 0.0;
  ros_raw_odom_msg.twist.vtheta = vth;
  
  ros_raw_odom_pub.publish(&ros_raw_odom_msg);
  
  // Remember current time for next publish
  last_raw_odom_publish_time = current_time;
}

/**
 * Converts accelerometer readings from LSM6 default settings
 * in m/s^2.
 */
geometry_msgs::Vector3 convertAccelerometer(const LSM6& imu) {
  // LSM6DS33 data sheet has accelerometer default
  // full scale setting with a conversion factor of
  // 0.061/LSB mg. Convert to g (* .001), 
  // and 1 g = 9.81 m/s^2.
  static const double conversionFactor(0.061 * 0.001 * 9.81);

  geometry_msgs::Vector3 converted;
  converted.x = imu.a.x * conversionFactor;
  converted.y = imu.a.y * conversionFactor;
  converted.z = imu.a.z * conversionFactor;
  return converted;
}

/**
 * Converts gyro readings from LSM6 default settings
 * in rad/sec.
 */
geometry_msgs::Vector3 convertGyro(const LSM6& imu) {
  // LSM6DS33 data sheet has accelerometer default
  // full scale setting with a conversion factor of
  // 0.061/LSB mg. Convert to g (* .001), 
  // and 1 g = 9.81 m/s^2.
  static const double conversionFactor(4.375 * 0.001 * 0.01745329251994);

  geometry_msgs::Vector3 converted;
  converted.x = imu.g.x * conversionFactor;
  converted.y = imu.g.y * conversionFactor;
  converted.z = imu.g.z * conversionFactor;
  return converted;
}

/**
 * Converts magnetometer readings from LIS3MDL
 * default settings to tesla.
 */
geometry_msgs::Vector3 convertMagnetometer(const LIS3MDL& mag) {
  // LIS3MDL data sheet, default full scale setting has
  // a conversion factor of LSB/6842 to get gauss value.
  // And 1 Tesla = 0.0001 gauss.
  static const double conversionFactor(6842 / 0.0001);

  geometry_msgs::Vector3 converted;
  converted.x = mag.m.x / conversionFactor;
  converted.y = mag.m.y / conversionFactor;
  converted.z = mag.m.z / conversionFactor;
  return converted;
}

/*
 * Publish imu data to ros.
 */
void doPublishRawImu() {
  ros::Time current_time = ros_nh.now();
  imu.read();
  mag.read();

  // Publish the imu message over ROS
  ros_raw_imu_msg.header.stamp = current_time;
  // TODO(mwomack): set header frame to what?

  ros_raw_imu_msg.accelerometer = convertAccelerometer(imu);
  ros_raw_imu_msg.gyro = convertGyro(imu);
  ros_raw_imu_msg.magnetometer = convertMagnetometer(mag);

  ros_raw_imu_pub.publish(&ros_raw_imu_msg);
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
  return total / NUM_VEL_SAMPLES;
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

void doBumperDebug() {
  // Read the bumper sensors
  snprintf(debug_str, sizeof(debug_str), 
    "L: %d    R: %d    STOP: %d", bumper_left, bumper_right, bumper_stop);
  ros_debug_msg.data = debug_str;
  ros_bumper_debug_pub.publish(&ros_debug_msg);
}

void doEncoderDebug() {
  // Read the encoders
  snprintf(debug_str, sizeof(debug_str), "L: %ld    R: %ld",
    encoder_left.read(), encoder_right.read());
  ros_debug_msg.data = debug_str;
  ros_encoder_debug_pub.publish(&ros_debug_msg);
}

/*
 * Converts a frequency in hertz to the
 * number of milliseconds to match the
 * frequency. For example, 2 hz = 500 millis
 * or 100 hz = 10 millis.
 */
int convertFreqToMillis(int frequency) {
  return static_cast<int>((1.0/frequency) * 1000);
}

/**
 * These normalize methods are copied from ros angles:
 * http://docs.ros.org/api/angles/html/angles_8h_source.html
 */
double normalize_angle_positive(double angle) {
  return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}

double normalize_angle(double angle) {
  double a = normalize_angle_positive(angle);
  if (a > M_PI) {
    a -= 2.0 *M_PI;
  }
  return a;
}

