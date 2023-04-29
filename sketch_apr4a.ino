#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <MPU6050.h>

#define REPORTING_PERIOD_MS     1000
int alcoholPin = A0;
int threshold = 500; // adjust this value depending on the sensitivity of the sensor
int warningThreshold = 800; // adjust this value depending on the desired warning level

// Create a PulseOximeter object
PulseOximeter pox;

// Time at which the last beat occurred
uint32_t tsLastReport = 0;

// Callback routine is executed when a pulse is detected
void onBeatDetected() {
    Serial.println("Beat!");
}

void setup() {
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
    // Read from the sensor
    pox.update();

    // Grab the updated heart rate and SpO2 levels
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println("%");

        tsLastReport = millis();
    }

    // mq 3

    int alcoholValue = analogRead(alcoholPin);
  if (alcoholValue > threshold) {
    Serial.println("Warning: High alcohol level detected!");
  }
  if (alcoholValue > warningThreshold) {
      Serial.println("Warning: Extremely high alcohol level detected!");
    }
  delay(100);
}