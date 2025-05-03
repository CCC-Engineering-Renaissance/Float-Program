#include <ArduinoJson.h>
// This library allows you to communicate with I2C devices
#include <Wire.h>
// Library for MS5837 pressure sensor
#include "MS5837.h"

MS5837 sensor;

// SensorData object used to store time, pressure and depth
struct SensorData {
  int time;
  float pressure;
  float depth;
};

// write to JSON document
void writeSensorData(const SensorData& data) {
  // set capacity of JsonDocument
  const size_t capacity = JSON_OBJECT_SIZE(3);
  // StaticJsonDocument is a JsonDocument that allocates its memory pool in-place
  // It doesn't rely on dynamic memory allocation (faster than DynamicJsonDocument)
  StaticJsonDocument<capacity> doc;

  //putting data into doc
  doc["time"] = data.time;
  doc["pressure"] = data.pressure;
  doc["depth"] = data.depth;

  // Printing doc to Serial monitor
  serializeJson(doc, Serial);
  Serial.println();
}

void setup() {
  // Initialize Serial port
  Serial.begin(9600);

  Serial.println("Starting");

  // initializes the Wire library and join the I2C bus as a controller or a peripheral
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  sensor.read();

  SensorData current;
  current.time = millis() / 1000;
  current.pressure = sensor.pressure();
  current.depth = sensor.depth();

  writeSensorData(current);

  delay(5000);
}
