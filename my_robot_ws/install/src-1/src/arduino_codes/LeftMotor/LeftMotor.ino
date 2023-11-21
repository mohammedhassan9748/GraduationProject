#include <ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

// Define motor control pins for the left and right wheels
int encoder_right_1 = 2;
int encoder_right_2 = 7;

int encoder_left_1 = 3;
int encoder_left_2 = 8;

volatile long right_encoderValue = 0;
volatile long left_encoderValue = 0;

ros::NodeHandle nh;
geometry_msgs::Point32 Point_msg;
ros::Publisher enc_pub("/encoder", &Point_msg);

void setup() {
    Serial.begin(57600);

  pinMode(encoder_right_1, INPUT_PULLUP); 
  pinMode(encoder_right_2, INPUT_PULLUP);

  pinMode(encoder_left_1, INPUT_PULLUP); 
  pinMode(encoder_left_2, INPUT_PULLUP);

  attachInterrupt(0, updateEncoder_right, RISING);
  attachInterrupt(1, updateEncoder_left, RISING);

    nh.advertise(enc_pub);
}

void loop() {
   
    Point_msg.x = right_encoderValue;
    Point_msg.y = left_encoderValue;
    enc_pub.publish(&Point_msg);

    nh.spinOnce();
    delay(10);    
}
void updateEncoder_right()
{
  
  int b = digitalRead(encoder_right_2); //LSB = least significant bit

  if(b==0) {right_encoderValue ++;}
  else{right_encoderValue --;} 

}

void updateEncoder_left()
{
  
  int b = digitalRead(encoder_left_2); //LSB = least significant bit

  if(b==0) {left_encoderValue --;}
  else{left_encoderValue ++;} 

}
