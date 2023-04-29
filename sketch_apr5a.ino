#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "MAX30100_PulseOximeter.h"

#define SIM800_RX 10
#define SIM800_TX 11

SoftwareSerial sim800(SIM800_RX, SIM800_TX);
TinyGPS gps;

// Initialize MPU6050 sensor
Adafruit_MPU6050 mpu;
float speed = 0;
float last_speed = 0;

// Initialize MQ-3 sensor
int mq3_pin = A0;
float alcohol_value = 0;

#define REPORTING_PERIOD_MS     1000

// Create a PulseOximeter object
PulseOximeter pox;

// Time at which the last beat occurred
uint32_t tsLastReport = 0;

// Last heart rate and SpO2 levels
uint8_t last_hr = 0;
uint8_t last_spo2 = 0;

// Callback routine is executed when a pulse is detected
void onBeatDetected() {
    Serial.println("Beat!");
}
  //void onBeatDetected() {
    //Serial.println("Beat!");
//}
void setup() {
    // Start serial communication
    Serial.begin(9600);
    sim800.begin(9600);

    // Initialize MPU6050 sensor
    mpu.begin();
    mpu.setAccelerometerRange(2);
    mpu.setGyroRange(250);

    // Initialize MQ-3 sensor
    pinMode(mq3_pin, INPUT);


    Serial.begin(9600);

    Serial.print("Initializing pulse oximeter..");

    // Initialize sensor
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

	// Configure sensor to use 7.6mA for LED drive
	pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback routine
    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
    // Read MPU6050 sensor values
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);
    float accel_x = accel_event.acceleration.x;
    float accel_y = accel_event.acceleration.y;
    float accel_z = accel_event.acceleration.z;

    // Calculate speed using MPU6050 sensor
    speed = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
    speed = speed * 3.6; // Convert to km/h

    // Read MQ-3 sensor value
    alcohol_value = analogRead(mq3_pin);

    // Read from the sensor
    pox.update();

    // Grab the updated heart rate and SpO2 levels
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        //Serial.print("Heart rate:");
        //Serial.print(pox.getHeartRate());
        //Serial.print("bpm / SpO2:");
       // Serial.print(pox.getSpO2());
       //Serial.println("%");

        // Check for sudden abnormalities in heart rate and SpO2

if ((pox.getHeartRate() > 100 && last_hr <= 100) || (pox.getSpO2() < 90 && last_spo2 >= 90)) {

// Send SMS message with warning
String message = "WARNING: Sudden abnormality detected in heart rate and/or SpO2! Please seek medical attention immediately.";
sim800.println("AT+CMGF=1"); // Set SMS mode to text
delay(100);
sim800.println("AT+CMGS=\"+919966127567\""); // Replace with phone number to send message to
delay(100);
sim800.print(message);
delay(100);
sim800.write(26); // Send message
Serial.println("Warning message sent.");
}

// Check for high alcohol value
if (alcohol_value > 500) {
    // Send SMS message with warning
    String message = "WARNING: High alcohol level detected! Please do not drive!";
    sim800.println("AT+CMGF=1"); // Set SMS mode to text
    delay(100);
    sim800.println("AT+CMGS=\"+919966127567\""); // Replace with phone number to send message to
    delay(100);
    sim800.print(message);
    delay(100);
    sim800.write(26); // Send message
    Serial.println(" High Alcohol Level Detected Warning message sent.");
}

// sending sms for speed variable  
  if (speed < 10.0) { // if sudden drop in speed is detected

            sim800.print("Sudden drop in speed detected! Current speed: ");
            sim800.print(speed);
            sim800.write(0x1A); // send the Ctrl+Z character to end the SMS
            delay(1000);

  }

// Update last values
last_hr = pox.getHeartRate();
last_spo2 = pox.getSpO2();

}
pox.update();

    // Grab the updated heart rate and SpO2 levels
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Speed (km/h): ");
        Serial.print(speed-300);
        Serial.print("\t Alcohol value: ");
        Serial.println(alcohol_value);
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println("%");

        tsLastReport = millis();
    }
// Print sensor values to serial monitor

//Serial.print("Heart rate: ");
//Serial.print(pox.getHeartRate() );
//Serial.print("bpm / SpO2: ");
//Serial.print(pox.getSpO2());
//Serial.println("%");

// Delay for a certain amount of time
delay(1000);



}
