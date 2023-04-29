
// working

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PI 3.14159
#define RADIUS 0.05 // radius of the wheel in meters
#define ACCEL_THRESHOLD 1.0 // acceleration threshold in m/s^2

Adafruit_MPU6050 mpu;

float acceleration, velocity = 0, displacement = 0;
unsigned long previousTime = 0;

int mq3Pin = A0; // analog input pin for MQ-3 sensor
int ledPin = 13; // LED pin for status indicator


void setup() {
  Serial.begin(9600);
  while (!Serial)
    delay(10);

  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  Serial.begin(9600); // initialize serial communication
  pinMode(ledPin, OUTPUT); // set LED pin as output  

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {


 int alcoholLevel = analogRead(mq3Pin); // read analog input value from MQ-3 sensor
  Serial.print("  Alcohol Level: ");
  Serial.println(alcoholLevel); // print alcohol level value to serial monitor
  
  if (alcoholLevel > 500) { // if alcohol level is above threshold
    digitalWrite(ledPin, HIGH); // turn on LED
  } else {
    digitalWrite(ledPin, LOW); // turn off LED
  }
  
//  delay(500); // wait for 500 milliseconds before taking next reading

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

  float speed = velocity * RADIUS * 3.6; // Convert to km/h
  Serial.print("Speed (km/h): ");
  Serial.print(speed);

  delay(50);
}
