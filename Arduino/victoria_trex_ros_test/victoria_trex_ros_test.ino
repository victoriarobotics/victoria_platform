#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

HardwareSerial trex = HardwareSerial();

boolean motor0Dir = true;
Encoder motor0Enc(29, 30);
long enc0Pos = 0;

boolean motor1Dir = true;
Encoder motor1Enc(31, 32);
long enc1Pos = 0;

boolean motorsOn = false;

void toggleMotors( const std_msgs::Empty& toggle_msg) {
  motorsOn = !motorsOn;
}

ros::Subscriber<std_msgs::Empty> toggleMotorsSub("toggle_motors", toggleMotors );

void toggleDirection( const std_msgs::Empty& toggle_msg) {
  motor0Dir = !motor0Dir;
  motor1Dir = !motor1Dir;
}

ros::Subscriber<std_msgs::Empty> toggleDirectionSub("toggle_direction", toggleDirection );


std_msgs::String encoder_msg;
ros::Publisher encoderInfoPub("encoder_info", &encoder_msg);

void setMotorSpeed(uint8_t motorIndex, boolean forward, uint8_t speed);


void setup() {
  //Serial.begin(57600);
  
  trex.begin(19200);

  nh.initNode();
  
  nh.advertise(encoderInfoPub);
  nh.subscribe(toggleMotorsSub);
  nh.subscribe(toggleDirectionSub);

  Serial.println("Setup complete");
}

boolean oneTime = false;
boolean stop = false;

void loop() {

  // publish current state
  String msg = "encoder 0: ";
  msg.concat(motor0Enc.read());
  msg.concat("; encoder 1: ");
  msg.concat(motor1Enc.read());
  encoder_msg.data = msg.c_str();
  encoderInfoPub.publish( &encoder_msg );
  
  nh.spinOnce();

  // handle state changes from subscribers
  if (motorsOn) {
    //Serial.println("Starting motors");
    setMotorSpeed(0, motor0Dir, 63);
    setMotorSpeed(1, motor1Dir, 63);
  } else {
    //Serial.println("Stopping motors");
    setMotorSpeed(0, motor0Dir, 0);
    setMotorSpeed(1, motor1Dir, 0);
  }

/*
  long newPos = motor0Enc.read();
  if (newPos != enc0Pos) {
    enc0Pos = newPos;
    //Serial.print("encoder 0: ");
    //Serial.println(enc0Pos);
  }
  newPos = motor1Enc.read();
  if (newPos != enc1Pos) {
    enc1Pos = newPos;
    //Serial.print("encoder 1: ");
    //Serial.println(enc1Pos);
  }
*/
}

void setMotorSpeed(uint8_t motorIndex, boolean forward, uint8_t speed) {
  //Serial.print("Setting motor speed...");
  //Serial.print(motorIndex);
  //Serial.print("...");
  //Serial.print(forward);
  //Serial.print("...");
  //Serial.println(speed);
  
  byte commandByte;
  if (motorIndex == 0) {
    commandByte = 0xC4;
  } else {
    commandByte = 0xCC;
  }
  
  if (forward) {
    commandByte = commandByte | 0x02;
  } else {
    commandByte = commandByte | 0x01;
  }
  
  //Serial.print("command: ");
  //Serial.println(commandByte, HEX);
  
  //Serial.print("speed: ");
  //Serial.println((byte)speed);
  
  trex.write(commandByte);
  trex.write((byte)speed);
}
