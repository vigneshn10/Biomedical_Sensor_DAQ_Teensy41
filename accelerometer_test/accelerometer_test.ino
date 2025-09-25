#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

// Create an instance of the LSM9DS1 sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Create a sensor event to hold the data
sensors_event_t a, g, m;

void setup() {
  Serial.begin(115200);
  
  // Initialize the sensor
  if (!lsm.begin()) {
    Serial.println("Failed to initialize LSM9DS1!");
    while (1);
  }
  
  // Set up accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);  // 2G is good for detecting heartbeats
  
  Serial.println("LSM9DS1 Initialized!");
}

void loop() {
  // Get the accelerometer, gyroscope, and magnetometer data
    lsm.read(); // Read sensor data

  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  
  // Print accelerometer data (use Z-axis for SCG)
  // Serial.print("Accel X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(" m/s^2\t");
  
  // Serial.print("Accel Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(" m/s^2\t");
  
  // Serial.print("Accel Z: ");
  Serial.println(a.acceleration.z);
  // Serial.println(" m/s^2");
  
  delay(10);  // Adjust delay for data sampling rate
}
