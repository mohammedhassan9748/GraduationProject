#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define LIFTING_TIME 5000

volatile int posi = 0; // specify posi as volatile

// Define motor pins
const int motorPin1 = 9;    // Motor control pin 1
const int motorPin2 = 10;  
 // Motor control pin 2
const int enablePin = 11;   // Motor enable  pin (PWM)
const int velocity  = 100;


// Define variables for motor direction and box state
bool motorDirection = false; // true: motor rotates clockwise, false: counterclockwise

ros::NodeHandle nh;

std_msgs::String status_msg;


void motors_cb(const std_msgs::String &msg)
{

  String mystring =String(msg.data); 
  if (mystring=="lift")
  {   
    motorDirection = true;
    // Control motor direction
    digitalWrite(motorPin1, !motorDirection);
    digitalWrite(motorPin2, motorDirection);
    analogWrite(enablePin, velocity);
  }  
  else if(mystring=="drop")
  {
    motorDirection = false;
    // Control motor direction
    digitalWrite(motorPin1, !motorDirection);
    digitalWrite(motorPin2, motorDirection);
    analogWrite(enablePin, velocity);
  } 
}
ros::Subscriber<std_msgs::String> sub("/chatter", motors_cb);
void setup()
{
  Serial.begin(57600);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

  nh.initNode();
  nh.advertise(status_pub);
  nh.subscribe(sub);
}

void loop()
{

  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  pos = posi;
  }
         // Wait for a certain amount of time (adjust as needed)
    if(abs(pos)>=(390*2) ){

    digitalWrite(motorPin1,LOW);
    digitalWrite(motorPin2,LOW);
    posi= 0; 
    }
  nh.spinOnce();
  delay(10); // Adjust as needed
}
void readEncoder(){

  int b = digitalRead(ENCB);

  if(b > 0){

    posi++;

  }

  else{

    posi--;

  }

}
