#include <ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

// Define motor control pins for the left and right wheels
#define leftMotorPWM   9  // PWM pin for left motor speed control
#define leftMotorDir   8  // Direction pin for left motor
#define rightMotorPWM  10 // PWM pin for right motor speed control
#define rightMotorDir  11 // Direction pin for right motor

// Robot parameters definition
#define L 0.37
#define R 0.07
#define max_linear_speed 0.26
#define max_angular_speed 1.82

// Initializing variables
float vel = 0.0; 
float omega = 0.0;
float VR = 0.0;
float VL = 0.0;

ros::NodeHandle nh;
void motors_cb(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", motors_cb);

void motors_cb(const geometry_msgs::Twist& msg) {
    vel = msg.linear.x;    
    omega = msg.angular.z;  

    VL = (vel - omega * L / 2) / R;
    VR = (vel + omega * L / 2) / R; 

    float maxWheelSpeed = (max_linear_speed + (max_angular_speed * L / 2)) / R;

    int PWM_L = mapFloatToPWM(VL, -maxWheelSpeed, maxWheelSpeed, -255, 255);
    int PWM_R = mapFloatToPWM(VR, -maxWheelSpeed, maxWheelSpeed, -255, 255);

    PWM_L = constrain(PWM_L, -255, 255);
    PWM_R = constrain(PWM_R, -255, 255);

    setMotorSpeed(leftMotorPWM, leftMotorDir, PWM_L);
    setMotorSpeed(rightMotorPWM, rightMotorDir, PWM_R);
     
}

void setMotorSpeed(int pwmPin, int dirPin, int speed) {
    digitalWrite(dirPin, speed < 0 ? HIGH : LOW);
    analogWrite(pwmPin, abs(speed));
}

int mapFloatToPWM(float x, float in_min, float in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
    Serial.begin(57600);

    pinMode(leftMotorPWM, OUTPUT);
    pinMode(leftMotorDir, OUTPUT);
    pinMode(rightMotorPWM, OUTPUT);
    pinMode(rightMotorDir, OUTPUT);

    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
    delay(10);
}
