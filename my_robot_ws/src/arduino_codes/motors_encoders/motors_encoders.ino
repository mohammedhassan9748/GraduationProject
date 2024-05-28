#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>

// Motor control pins
#define leftMotorPWM 9
#define leftMotorDir 8
#define rightMotorPWM 10
#define rightMotorDir 11

// Encoder pins
#define encoder_right_1 2
#define encoder_right_2 7
#define encoder_left_1 3
#define encoder_left_2 6

// Robot parameters
#define L 0.37
#define R 0.07
#define N 360.0

// PID constants
#define Kp 1.0
#define Ki 0.1
#define Kd 0.01

// Variables
float vel = 0.0;
float omega = 0.0;
float VR = 0.0;
float VL = 0.0;
volatile long right_encoderValue = 0;
volatile long left_encoderValue = 0;
float right_actual_velocity = 0.0;
float left_actual_velocity = 0.0;
float right_error_sum = 0.0;
float left_error_sum = 0.0;
float right_last_error = 0.0;
float left_last_error = 0.0;

ros::NodeHandle nh;

void motors_cb(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", motors_cb);

geometry_msgs::Point32 Point_msg;
ros::Publisher enc_pub("/encoder", &Point_msg);

void motors_cb(const geometry_msgs::Twist& msg) {
    vel = msg.linear.x;
    omega = msg.angular.z;

    // Calculate desired wheel velocities
    VL = (vel - (omega * L / 2));
    VR = (vel + (omega * L / 2));
}

void setMotorSpeed(int pwmPin, int dirPin, float velocity, float actual_velocity, float &error_sum, float &last_error) {
    float error = velocity - actual_velocity;
    error_sum += error;
    float d_error = error - last_error;

    float output = Kp * error + Ki * error_sum + Kd * d_error;

    int pwmValue = constrain(static_cast<int>(abs(output)), 0, 255);
    int direction = output < 0 ? HIGH : LOW;
    digitalWrite(dirPin, direction);
    analogWrite(pwmPin, pwmValue);

    last_error = error;
}

void updateEncoder_right() {
    int b = digitalRead(encoder_right_2);
    if (b == 0) {
        right_encoderValue++;
    } else {
        right_encoderValue--;
    }
}

void updateEncoder_left() {
    int b = digitalRead(encoder_left_2);
    if (b == 0) {
        left_encoderValue--;
    } else {
        left_encoderValue++;
    }
}

void setup() {
    Serial.begin(57600);
    pinMode(leftMotorPWM, OUTPUT);
    pinMode(leftMotorDir, OUTPUT);
    pinMode(rightMotorPWM, OUTPUT);
    pinMode(rightMotorDir, OUTPUT);

    pinMode(encoder_right_1, INPUT_PULLUP);
    pinMode(encoder_right_2, INPUT_PULLUP);
    pinMode(encoder_left_1, INPUT_PULLUP);
    pinMode(encoder_left_2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoder_right_1), updateEncoder_right, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_left_1), updateEncoder_left, RISING);

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(enc_pub);
}

void loop() {
    static long last_right_encoderValue = 0;
    static long last_left_encoderValue = 0;
    static unsigned long last_time = millis();

    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;

    if (dt > 0) {
        long right_ticks = right_encoderValue - last_right_encoderValue;
        long left_ticks = left_encoderValue - last_left_encoderValue;

        right_actual_velocity = (2 * PI * R * right_ticks) / (N * dt);
        left_actual_velocity = (2 * PI * R * left_ticks) / (N * dt);

        last_right_encoderValue = right_encoderValue;
        last_left_encoderValue = left_encoderValue;
        last_time = current_time;
    }

    setMotorSpeed(leftMotorPWM, leftMotorDir, VL, left_actual_velocity, left_error_sum, left_last_error);
    setMotorSpeed(rightMotorPWM, rightMotorDir, VR, right_actual_velocity, right_error_sum, right_last_error);

    Point_msg.x = right_encoderValue;
    Point_msg.y = left_encoderValue;
    enc_pub.publish(&Point_msg);

    nh.spinOnce();
    delay(10);
}
