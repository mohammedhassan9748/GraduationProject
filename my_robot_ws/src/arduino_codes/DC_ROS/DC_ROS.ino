#include <ros.h>
#include <geometry_msgs/Point32.h> //For sending encoder msg
#include <geometry_msgs/Twist.h> //For cmd_vel subscription

// Define motor control pins for the left and right wheels
const int leftMotorPWM = 9;  // PWM pin for left motor speed control
const int leftMotorDir1 = 8; // Direction pin 1 for left motor
const int leftMotorDir2 = 7; // Direction pin 2 for left motor
const int rightMotorPWM = 10; // PWM pin for right motor speed control
const int rightMotorDir1 = 11; // Direction pin 1 for right motor
const int rightMotorDir2 = 12; // Direction pin 2 for right motor

// Robot parameters definition
#define L 0.133
#define R 0.034

// Initializing variables
float vel = 0.0; 
float omega = 0.0;
float VR, VL;

// Encoder variables
int counterL = 0;
int counterR = 0;
const int leftmotorencoder = 4; // Left Motor encoder
const int rightmotorencoder = 3; // Right Motor encoder
long RoldPosition = -999;
long LoldPosition = -999;
long LnewPosition = 0;
long RnewPosition = 0;

ros::NodeHandle nh;

geometry_msgs::Point32 Point_msg;
ros::Publisher enc_pub("/encoder", &Point_msg); 
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", motors_cb);

void motors_cb(const geometry_msgs::Twist& msg) {
    vel = msg.linear.x;    
    omega = msg.angular.z;  

    // Calculate wheel velocities in meters per second
    VR = (vel + omega * L / 2) / R; 
    VL = (vel - omega * L / 2) / R;

    // Maximum wheel velocities based on the robot's max linear and angular velocities
    float maxWheelSpeed = 0.22 + (2.84 * L / 2) / R; // Maximum possible wheel speed

    // Convert wheel velocities to PWM values
    int PWM_R = mapFloatToPWM(VR, -maxWheelSpeed, maxWheelSpeed, -255, 255);
    int PWM_L = mapFloatToPWM(VL, -maxWheelSpeed, maxWheelSpeed, -255, 255);

    // Ensure PWM values are within bounds
    PWM_R = constrain(PWM_R, -255, 255);
    PWM_L = constrain(PWM_L, -255, 255);

    // Set motor directions and speeds based on PWM values
    setMotorSpeed(leftMotorPWM, leftMotorDir1, leftMotorDir2, PWM_L);
    setMotorSpeed(rightMotorPWM, rightMotorDir1, rightMotorDir2, PWM_R);
}

void setMotorSpeed(int pwmPin, int dirPin1, int dirPin2, int speed) {
    if (speed < 0) {
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
    } else {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
    }
    analogWrite(pwmPin, abs(speed));
}

int mapFloatToPWM(float x, float in_min, float in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
    Serial.begin(57600);

    // Set motor control pins as outputs
    pinMode(leftMotorPWM, OUTPUT);
    pinMode(leftMotorDir1, OUTPUT);
    pinMode(leftMotorDir2, OUTPUT);
    pinMode(rightMotorPWM, OUTPUT);
    pinMode(rightMotorDir1, OUTPUT);
    pinMode(rightMotorDir2, OUTPUT);

    pinMode(leftmotorencoder, INPUT);
    pinMode(rightmotorencoder, INPUT);
    
    LnewPosition=digitalRead(leftmotorencoder);
    LoldPosition=LnewPosition;
    RnewPosition=digitalRead(rightmotorencoder);
    RoldPosition=RnewPosition;
    
    nh.advertise(enc_pub);
    nh.subscribe(sub);
}

void loop() {
    // Encoder logic and ROS node spinning
    
    // Right Encoder
    RnewPosition = digitalRead(rightmotorencoder);
    if (RnewPosition != RoldPosition) {
      RoldPosition = RnewPosition;
      if(RnewPosition != 1){ 
       counterR +=1;}
    } 
    // Left encoder
    LnewPosition = digitalRead(leftmotorencoder);
    if (LnewPosition != LoldPosition){ 
      LoldPosition = LnewPosition; //update positions
      if(LnewPosition != 1){counterL +=1;}
    }
    
    // Update encoder readings, publish encoder data, and handle ROS callbacks
    Point_msg.x=counterR;
    Point_msg.y=counterL;
    enc_pub.publish(&Point_msg);
    
    nh.spinOnce();
    delay(10);
}
