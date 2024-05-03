#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <dingo_nano_interfacing/ElectricalMeasurements.h>
#include <std_msgs/String.h>  // Use simple string messages for GPS data
#include <avr/sleep.h>
#include <SoftwareSerial.h>

SoftwareSerial gps(4, 3);  // RX, TX
char gpsData[256];  // Buffer to store GPS data
int index = 0;  // Index for gpsData array

ros::NodeHandle_<ArduinoHardware, 5, 5, 128, 256> arduino_nano_node;

std_msgs::Bool emergency_stop_msg;
dingo_nano_interfacing::ElectricalMeasurements electrical_measurement_msg;
std_msgs::String gps_msg;  // ROS message for GPS data

ros::Publisher emergency_stop_status_publisher("emergency_stop_status", &emergency_stop_msg);
ros::Publisher electrical_measurement_publisher("electrical_measurements", &electrical_measurement_msg);
ros::Publisher gps_publisher("gps_data", &gps_msg);

void setup() {
  Serial.begin(115200);
  gps.begin(9600);  // Initialize GPS
  arduino_nano_node.initNode();
  arduino_nano_node.advertise(emergency_stop_status_publisher);
  arduino_nano_node.advertise(electrical_measurement_publisher);
  arduino_nano_node.advertise(gps_publisher);
}

void loop() {
  checkBatteryVoltageLevel();
  checkBuckVoltageLevel();
  electrical_measurement_publisher.publish(&electrical_measurement_msg);
  
  // Read GPS data
  while (gps.available()) {
    char c = gps.read();
    if (c == '\n') {
      gpsData[index] = '\0';  // Null-terminate the string
      gps_msg.data = gpsData;  // Set the ROS message data
      gps_publisher.publish(&gps_msg);  // Publish GPS data
      index = 0;  // Reset the index
    } else {
      gpsData[index++] = c;  // Store character in buffer
    }
  }
  
  arduino_nano_node.spinOnce();
  delay(100);
}

void checkBatteryVoltageLevel() {
  const float minVoltage = 11.0;  // Minimum operational voltage for battery
  int sensorValue = analogRead(A0);  // Reading from analog pin A0
  float batteryVoltage = sensorValue * (5.0 / 1023.0) * 11;  // Convert to actual voltage
  electrical_measurement_msg.battery_voltage_level = batteryVoltage;

  // If battery voltage is too low, trigger emergency stop
  if (batteryVoltage < minVoltage) {
    emergency_stop_msg.data = true;
    emergency_stop_status_publisher.publish(&emergency_stop_msg);
    shutdown();
  }
}

void checkBuckVoltageLevel() {
  const float minBuckVoltage = 5.0;  // Minimum operational voltage for buck converter
  int sensorValue = analogRead(A1);  // Reading from analog pin A1
  float buckVoltage = sensorValue * (5.0 / 1023.0) * 10;  // Convert to actual voltage
  electrical_measurement_msg.servo_buck_voltage_level = buckVoltage;

  // Check if buck voltage is too low, could indicate a failure
  if (buckVoltage < minBuckVoltage) {
    emergency_stop_msg.data = true;
    emergency_stop_status_publisher.publish(&emergency_stop_msg);
  }
}

void shutdown() {
  // Logs and shuts down the system
  arduino_nano_node.logwarn("SYSTEM SHUTDOWN: LOW BATTERY");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  sleep_cpu();  // Put Arduino to sleep
}
