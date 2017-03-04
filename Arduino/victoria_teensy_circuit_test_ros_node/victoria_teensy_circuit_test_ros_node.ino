// Teensy includes
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <i2c_t3.h>
#include <LSM6.h>
#include <LIS3MDL.h>

// ROS includes
#include <ros.h>
#include <std_msgs/String.h>

// Teensy pin definitions
#define ENCODER_LEFT_PIN_1 29   // Teensy digital pin 29
#define ENCODER_LEFT_PIN_2 30   // Teensy digital pin 30
#define ENCODER_RIGHT_PIN_1 31  // Teensy digital pin 31
#define ENCODER_RIGHT_PIN_2 32  // Teensy digital pin 32
#define BUMPER_LEFT_PIN 14      // Teensy analog pin 14
#define BUMPER_RIGHT_PIN 15     // Teensy analog pin 15

// TRex motor controller
HardwareSerial trex = HardwareSerial();

// Motor encoders
Encoder encoder_left(ENCODER_LEFT_PIN_1, ENCODER_LEFT_PIN_2);
Encoder encoder_right(ENCODER_RIGHT_PIN_1, ENCODER_RIGHT_PIN_2);

// IMU and Magnetometer
LSM6 imu;
LIS3MDL mag;
bool imu_err = false;
bool mag_err = false;

char debug_str[80];

std_msgs::String ros_debug_msg;
ros::Publisher ros_bumper_pub("bumper_info", &ros_debug_msg);
ros::Publisher ros_encoder_pub("encoder_info", &ros_debug_msg);
ros::Publisher ros_imu_pub("imu_info", &ros_debug_msg);
ros::Publisher ros_magnetometer_pub("magnetometer_info", &ros_debug_msg);

// ROS node handle
ros::NodeHandle ros_nh;

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

  mag.enableDefault();
    
  // Start serial port to TRex
  trex.begin(19200);

  // Initialize ros node
  ros_nh.initNode();

  // Initialize ROS publishers
  ros_nh.advertise(ros_bumper_pub);
  ros_nh.advertise(ros_encoder_pub);
  ros_nh.advertise(ros_imu_pub);
  ros_nh.advertise(ros_magnetometer_pub);
}

void loop() {
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

  // Send all the ROS messages
  ros_nh.spinOnce();
}
