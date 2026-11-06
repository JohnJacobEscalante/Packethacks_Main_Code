#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "INA226.h"

``
// ERSM-1 CORE FIRMWARE
// RAK3112 / Cubicore + INA226


// I2C pins for Cubicore
#define SDA_PIN 9
#define SCL_PIN 40

// INA226
#define INA226_ADDRESS 0x40
const float SHUNT_OHMS = 0.010;
const float CALIBRATION_MAX_CURRENT = 3.5;

// Relay configuration
// Keep -1 until the actual relay GPIO is verified.
#define RELAY_PIN -1

// Change after testing the actual relay module.
#define RELAY_ACTIVE_LOW true

// Sensor plausibility limits, NOT safety trip settings.
const float MAX_VALID_VOLTAGE = 25.0;
const float MAX_VALID_CURRENT = 4.0;

// Optional operating protection.
// Leave disabled until the actual load and protection
// requirements have been verified.
const float CURRENT_TRIP_LIMIT = 0.0;

// Monitoring interval
const unsigned long SAMPLE_INTERVAL_MS = 100;

// Initialization retries
const int INA_MAX_RETRIES = 3;

// Telemetry buffer
const int BUFFER_SIZE = 64;

struct Telemetry {
  unsigned long timestamp;
  float voltage;
  float current;
  float power;
  float energyWh;
};

Telemetry dataBuffer[BUFFER_SIZE];

int bufferHead = 0;
int bufferTail = 0;
int bufferCount = 0;
unsigned long droppedSamples = 0;

// Sensor and system state
INA226 ina(INA226_ADDRESS);

bool inaReady = false;
bool sensorFault = false;
bool relayState = false;

unsigned long lastSampleTime = 0;
unsigned long previousEnergyTime = 0;

float accumulatedEnergyWh = 0.0;


// RELAY CONTROL


void setRelay(bool turnOn) {

  // No relay GPIO has been assigned yet.
  if (RELAY_PIN < 0) {
    relayState = false;
    return;
  }

  bool outputLevel;

  if (RELAY_ACTIVE_LOW) {
    outputLevel = !turnOn;
  } else {
    outputLevel = turnOn;
  }

  digitalWrite(RELAY_PIN, outputLevel);
  relayState = turnOn;
}


// TELEMETRY BUFFER

void storeTelemetry(
  float voltage,
  float current,
  float power
) {

  // Do not overwrite untransmitted measurements.
  if (bufferCount >= BUFFER_SIZE) {
    droppedSamples++;
    return;
  }

  dataBuffer[bufferHead].timestamp = millis();
  dataBuffer[bufferHead].voltage = voltage;
  dataBuffer[bufferHead].current = current;
  dataBuffer[bufferHead].power = power;
  dataBuffer[bufferHead].energyWh = accumulatedEnergyWh;

  bufferHead = (bufferHead + 1) % BUFFER_SIZE;
  bufferCount++;
}

// Call this ONLY after the gateway confirms successful
// delivery of the corresponding telemetry record.
void removeConfirmedTelemetry() {

  if (bufferCount <= 0) {
    return;
  }

  bufferTail = (bufferTail + 1) % BUFFER_SIZE;
  bufferCount--;
}


// ENERGY CALCULATION


void updateEnergy(float power) {

  unsigned long now = millis();

  if (previousEnergyTime != 0) {

    unsigned long elapsedMs =
      now - previousEnergyTime;

    float elapsedHours =
      elapsedMs / 3600000.0;

    // Accumulate only positive generated power.
    if (power > 0) {
      accumulatedEnergyWh += power * elapsedHours;
    }
  }

  previousEnergyTime = now;
}


// INA226 INITIALIZATION


bool initializeINA226() {

  Wire.begin(SDA_PIN, SCL_PIN);

  for (int attempt = 1;
       attempt <= INA_MAX_RETRIES;
       attempt++) {

    Serial.print("INA226 initialization attempt ");
    Serial.print(attempt);
    Serial.println(" of 3");

    if (ina.begin()) {

      Serial.println("INA226 detected!");

      int result = ina.setMaxCurrentShunt(
        CALIBRATION_MAX_CURRENT,
        SHUNT_OHMS
      );

      if (result == 0) {

        Serial.println("INA226 calibration successful!");

        delay(100);

        return true;
      }

      Serial.print("Calibration error: 0x");
      Serial.println(result, HEX);
    }

    delay(1000);
  }

  Serial.println("INA226 initialization FAILED!");

  return false;
}

// READING VALIDATION

bool validReading(
  float voltage,
  float current,
  float power,
  float shuntVoltage
) {

  if (!isfinite(voltage) ||
      !isfinite(current) ||
      !isfinite(power) ||
      !isfinite(shuntVoltage)) {
    return false;
  }

  if (voltage < 0 ||
      voltage > MAX_VALID_VOLTAGE) {
    return false;
  }

  if (fabs(current) > MAX_VALID_CURRENT) {
    return false;
  }

  if (fabs(shuntVoltage) > 0.08192) {
    return false;
  }

  return true;
}

// ==========================================
// MONITORING
// ==========================================

void monitorSolarSystem() {

  float voltage = ina.getBusVoltage();
  float current = ina.getCurrent();
  float power = ina.getPower();
