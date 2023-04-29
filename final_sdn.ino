// working

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30100_PulseOximeter.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

PulseOximeter pox;

#define REPORTING_PERIOD_MS 1000

#define GPS_TX_PIN 7 // GPS module's TX pin
#define GPS_RX_PIN 8 // GPS module's RX pin

SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);

#define SIM800_TX_PIN 10 // SIM800L's TX pin
#define SIM800_RX_PIN 11 // SIM800L's RX pin

SoftwareSerial sim800Serial(SIM800_TX_PIN, SIM800_RX_PIN);

TinyGPSPlus gps;

Adafruit_MPU6050 mpu;

float acceleration, velocity = 0, displacement = 0;
unsigned long previousTime = 0;

int mq3Pin = A0; // analog input pin for MQ-3 sensor
int ledPin = 13; // LED pin for status indicator

#define PI 3.14159
#define RADIUS 0.05 // radius of the wheel in meters
#define ACCEL_THRESHOLD 10.0 // acceleration threshold in m/s^2
#define PHONE_NUMBER "+919966127567" // Phone number to send the SMS to



void setup() {
  Serial.begin(9600);
  while (!Serial); // wait for serial monitor to open

  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  pinMode(ledPin, OUTPUT); // set LED pin as output  

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  gpsSerial.begin(9600);
  sim800Serial.begin(9600);

  if (!pox.begin()) {
    Serial.println("MAX30100 was not found. Please check wiring/power.");
    while (1);
  }
}
void loop() {
  static uint32_t lastReportTime = 0;
  float heartRate, oxygenSaturation, speed;
  int alcoholLevel;

  pox.update();
  heartRate = pox.getHeartRate();
  oxygenSaturation = pox.getSpO2();

  sensors_event_t a, g;
  mpu.getEvent(&a, &g, NULL);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  acceleration = sqrt(ax * ax + ay * ay + az * az);

  if (acceleration > ACCEL_THRESHOLD) {
    unsigned long currentTime = millis();
    float timeInterval = (currentTime - previousTime) / 1000.0;

    velocity += acceleration * timeInterval;
    displacement += velocity * timeInterval;

    previousTime = currentTime;
  }

  speed = velocity * RADIUS * 3.6; // Convert to km/h
  Serial.print("Speed (km/h): ");
  Serial.print(speed);

  alcoholLevel = analogRead(mq3Pin); // read analog input value from MQ-3 sensor
  Serial.print("  Alcohol Level: ");
  Serial.println(alcoholLevel); // print alcohol level value to serial monitor

  if (millis() - lastReportTime > REPORTING_PERIOD_MS) {
    lastReportTime = millis();

    if (heartRate > 0 && heartRate < 100) {
      Serial.print("Heart rate: ");
      Serial.print(heartRate, 1);
      Serial.println(" bpm");
    }

    if (oxygenSaturation > 0 && oxygenSaturation < 100) {
      Serial.print("Oxygen saturation: ");
      Serial.print(oxygenSaturation, 1);
      Serial.println("%");
    }
  }




// sending sms for speed variable  
  if (speed < 10.0) { // if sudden drop in speed is detected
    digitalWrite(ledPin, HIGH);
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
            sim800Serial.println("AT+CMGF=1"); // set SMS mode to text
            delay(1000);
            sim800Serial.println("AT+CMGS=\"" PHONE_NUMBER "\""); // set the phone number to send SMS to
            delay(1000);
            sim800Serial.print("Sudden drop in speed detected! Current speed: ");
            sim800Serial.print(speed);
            sim800Serial.write(0x1A); // send the Ctrl+Z character to end the SMS
            delay(1000);
            digitalWrite(ledPin, LOW);
  }}}}

// sending sms for heart rate
      if (heartRate > 60 && heartRate < 100) {
        Serial.println("Normal heart rate");
      } else if (heartRate <= 60) {
        Serial.println("Low heart rate. Alerting emergency contact.");

        // Get GPS location
        while (gpsSerial.available() > 0) {
          if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
              // Send location to emergency contact
              sim800Serial.println("AT+CMGF=1"); // Set message format to text
              delay(1000);
              sim800Serial.print("AT+CMGS=\""); // Send message to contact number
              sim800Serial.print("+919966127567"); // the contact number
              sim800Serial.println("\"");
              delay(1000);
              sim800Serial.print("Latitude: ");
              sim800Serial.println(gps.location.lat(), 6);
              sim800Serial.print("Longitude: ");
              sim800Serial.println(gps.location.lng(), 6);
              delay(1000);
              sim800Serial.write(26);
              delay(1000);
            }
          }
        }
      } else if (heartRate >= 100) {
        Serial.println("High heart rate. Alerting emergency contact.");
        
        // Get GPS location
        while (gpsSerial.available() > 0) {
          if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
              // Send location to emergency contact
              sim800Serial.println("AT+CMGF=1"); // Set message format to text
              delay(1000);
              sim800Serial.print("AT+CMGS=\""); // Send message to contact number
              sim800Serial.print("+919966127567"); // Replace with the contact number
                    }
                  }
                }
              } else {
                 Serial.println("No heart rate detected.");
                      }
  delay(1000);
}
    

