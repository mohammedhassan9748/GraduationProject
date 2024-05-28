#include <ros.h>
#include <geometry_msgs/Twist.h>

// Motor control pins
#define leftMotorPWM   9
#define leftMotorDir   8
#define rightMotorPWM  10
#define rightMotorDir  11

// Robot parameters
#define L 0.37
// Calibrated maximum speeds based on readings
#define max_wheel_speed 0.56      

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

    // Calculate wheel velocities
    VL = (vel - (omega * L / 2));
    VR = (vel + (omega * L / 2)); 

    // Map wheel speeds to PWM values and set motor speeds
    setMotorSpeed(leftMotorPWM, leftMotorDir, VL);
    setMotorSpeed(rightMotorPWM, rightMotorDir, VR);
}

void setMotorSpeed(int pwmPin, int dirPin, float velocity) {
    int direction = velocity < 0 ? HIGH : LOW;
    int pwmValue = mapVelocityToPWM(abs(velocity));

    digitalWrite(dirPin, direction);
    analogWrite(pwmPin, pwmValue);
}

int mapVelocityToPWM(float velocity) {
    // Use absolute value for PWM calculation to ensure positive values
    float abs_velocity = abs(velocity);

    // Cubic coefficients from curve fitting
    float a = 338.72, b = -239.50, c = 453.80, d = 23.80;
    int pwm = a * abs_velocity * abs_velocity * abs_velocity + b * abs_velocity * abs_velocity + c * abs_velocity + d;

    return constrain(pwm, 0, 255);
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
