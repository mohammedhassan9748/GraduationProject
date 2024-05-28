#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// MOSFET gate control pin
#define mosfetGatePin  12 

// Define ADC pins for battery voltage
#define batteryPin1 A0
#define batteryPin2 A1
#define batteryPin3 A2

ros::NodeHandle nh;
std_msgs::String voltage_msg;
ros::Publisher voltage_pub("/battery_voltage", &voltage_msg);
ros::Subscriber<std_msgs::Bool> mosfet_sub("/mosfet_state", mosfet_cb);

char voltage_str[50]; // Buffer to hold voltage value as string

void mosfet_cb(const std_msgs::Bool& msg) {
    digitalWrite(mosfetGatePin, msg.data ? HIGH : LOW);
}

void setup() {
    Serial.begin(57600);
    
    for (int pin = A0; pin <= A2; pin++) {
        pinMode(pin, INPUT);
    }

    nh.initNode();
    nh.advertise(voltage_pub);
    nh.subscribe(mosfet_sub);
}

void loop() {
    float prevVoltage = 0;
    String voltageReadings = "";
    float maxVoltage = 0.0;
    for (int pin = A0; pin <= A2; pin++) {
        float cellMaxVoltage = (pin == A0) ? 5.0 : 4.2 + (pin - A0) * 4.2;
        float voltage = readBatteryVoltage(pin, cellMaxVoltage) - prevVoltage;
        prevVoltage += voltage;
        voltageReadings += String(voltage, 2) + ", ";
    }
    voltageReadings += String(prevVoltage, 2) + ", ";
    voltageReadings.toCharArray(voltage_str, 50);
    voltage_msg.data = voltage_str;
    voltage_pub.publish(&voltage_msg);

    nh.spinOnce();
    delay(10);
}

// Function to convert ADC value to battery voltage
float readBatteryVoltage(int pin, float maxVoltage) {
    const float adcVoltageMax = 5.0; // Maximum voltage the ADC can read
    return (analogRead(pin) * maxVoltage / 1023.0);
}
