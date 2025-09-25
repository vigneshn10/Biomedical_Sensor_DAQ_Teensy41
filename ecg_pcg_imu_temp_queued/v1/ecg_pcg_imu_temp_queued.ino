#include <Arduino.h>
#include <Wire.h>
#include <TeensyThreads.h>
#include <SparkFunLSM9DS1.h>
#include "Protocentral_MAX30205.h"
#include "protocentral_Max30003.h"
#include <Filters.h>
#include <queue>

// ========== PIN ASSIGNMENTS (Optimized for Teensy 4.1) ==========
// I2C Bus (Wire - Primary I2C)
#define I2C_SDA_PIN 18  // Default SDA
#define I2C_SCL_PIN 19  // Default SCL

// PCG Analog Input
#define PCG_PIN A2  // Pin 16 (ADC1_IN11)

// MAX30003 ECG (SPI)
#define ECG_CS_PIN 10    // Chip Select
#define ECG_MOSI_PIN 11  // MOSI (default SPI)
#define ECG_MISO_PIN 12  // MISO (default SPI)
#define ECG_SCK_PIN 13   // SCK (default SPI)
#define ECG_INT_PIN 9    // Interrupt pin

// Status LED
#define LED_PIN LED_BUILTIN

// ========== TIMING CONFIGURATION ==========
#define MASTER_SAMPLE_RATE_HZ 1000  // 1 kHz master clock
#define MASTER_PERIOD_US (1000000 / MASTER_SAMPLE_RATE_HZ)

// ========== PCG FILTER SETUP ==========
#define PCG_CUTOFF_HZ 150.0f
#define PCG_SAMPLE_RATE 1000.0f
#define PCG_ORDER 2
FilterOnePole lowPassFilter(LOWPASS, PCG_CUTOFF_HZ / (PCG_SAMPLE_RATE / 2.0));

// ========== PCG CONFIGURATION ==========
#define PCG_VREF 3.3f
#define PCG_ADC_MAX 4095
#define PCG_DC_OFFSET 1550
#define PCG_SOFTWARE_GAIN 10

// ========== SENSOR INSTANCES ==========
LSM9DS1 imu;
MAX30205 tempSensor;
MAX30003 ecgSensor;

// ========== SENSOR ADDRESSES ==========
#define LSM9DS1_M 0x1E
#define LSM9DS1_AG 0x6B
#define MAX30205_ADDR 0x48

// ========== DATA STRUCTURES ==========
struct PCGData {
  uint32_t timestamp;
  int16_t value;
};

struct AccelData {
  uint32_t timestamp;
  float x, y, z;
};

struct TempData {
  uint32_t timestamp;
  float temperature;
};

struct ECGData {
  uint32_t timestamp;
  int32_t value;
};

// ========== GLOBAL QUEUES ==========
std::queue<PCGData> pcgQueue;
std::queue<AccelData> accelQueue;
std::queue<TempData> tempQueue;
std::queue<ECGData> ecgQueue;

// ========== THREAD-SAFE QUEUE ACCESS ==========
Threads::Mutex pcgMutex;
Threads::Mutex accelMutex;
Threads::Mutex tempMutex;
Threads::Mutex ecgMutex;

// ========== SENSOR STATUS FLAGS ==========
volatile bool pcgReady = false;
volatile bool accelReady = false;
volatile bool tempReady = false;
volatile bool ecgReady = false;

// ========== GLOBAL TIMER ==========
IntervalTimer masterTimer;
volatile uint32_t masterTimestamp = 0;

void masterTimerISR() {
  masterTimestamp++;
}

// ========== SENSOR VALIDATION FLAGS ==========
volatile bool sensorsValidated = false;
volatile bool systemHalted = false;
volatile uint8_t sensorErrors = 0x00;  // Bit flags: 0=PCG, 1=Accel, 2=Temp, 3=ECG

#define ERROR_PCG 0x01
#define ERROR_ACCEL 0x02
#define ERROR_TEMP 0x04
#define ERROR_ECG 0x08

// ========== WATCHDOG COUNTERS ==========
volatile uint32_t pcgLastUpdate = 0;
volatile uint32_t accelLastUpdate = 0;
volatile uint32_t tempLastUpdate = 0;
volatile uint32_t ecgLastUpdate = 0;

#define WATCHDOG_TIMEOUT_MS 5000  // 5 seconds without data = failure

// ========== PCG THREAD ==========
void pcgThread() {
  uint32_t lastSample = 0;
  
  while (true) {
    if (systemHalted) {
      threads.delay(1000);
      continue;
    }
    
    uint32_t currentTime = masterTimestamp;
    
    if (currentTime - lastSample >= 1) {  // 1ms sampling
      lastSample = currentTime;
      
      uint16_t raw = analogRead(PCG_PIN);
      int16_t centered = static_cast<int16_t>(raw) - PCG_DC_OFFSET;
      float amplified = centered * PCG_SOFTWARE_GAIN;
      float filtered = lowPassFilter.input(amplified);
      
      PCGData data;
      data.timestamp = currentTime;
      data.value = (int16_t)filtered;
      
      pcgMutex.lock();
      pcgQueue.push(data);
      if (pcgQueue.size() > 100) pcgQueue.pop();  // Prevent overflow
      pcgMutex.unlock();
      
      pcgLastUpdate = millis();
    }
    
    threads.yield();
  }
}

// ========== ACCELEROMETER THREAD ==========
void accelThread() {
  uint32_t lastSample = 0;
  
  while (true) {
    if (systemHalted) {
      threads.delay(1000);
      continue;
    }
    
    uint32_t currentTime = masterTimestamp;
    
    if (currentTime - lastSample >= 1) {  // 1ms sampling
      lastSample = currentTime;
      
      if (imu.accelAvailable()) {
        imu.readAccel();
        
        AccelData data;
        data.timestamp = currentTime;
        data.x = imu.calcAccel(imu.ax);
        data.y = imu.calcAccel(imu.ay);
        data.z = imu.calcAccel(imu.az);
        
        accelMutex.lock();
        accelQueue.push(data);
        if (accelQueue.size() > 100) accelQueue.pop();
        accelMutex.unlock();
        
        accelLastUpdate = millis();
      }
    }
    
    threads.yield();
  }
}

// ========== TEMPERATURE THREAD ==========
void tempThread() {
  uint32_t lastSample = 0;
  
  while (true) {
    if (systemHalted) {
      threads.delay(1000);
      continue;
    }
    
    uint32_t currentTime = masterTimestamp;
    
    if (currentTime - lastSample >= 1000) {  // 1Hz sampling (1000ms)
      lastSample = currentTime;
      
      if (tempReady) {
        float temp = tempSensor.getTemperature();
        
        if (!isnan(temp) && temp > -40 && temp < 85) {
          TempData data;
          data.timestamp = currentTime;
          data.temperature = temp;
          
          tempMutex.lock();
          tempQueue.push(data);
          if (tempQueue.size() > 10) tempQueue.pop();
          tempMutex.unlock();
          
          tempLastUpdate = millis();
        }
      }
    }
    
    threads.yield();
  }
}

// ========== ECG THREAD ==========
void ecgThread() {
  uint32_t lastSample = 0;
  
  while (true) {
    if (systemHalted) {
      threads.delay(1000);
      continue;
    }
    
    uint32_t currentTime = masterTimestamp;
    
    if (currentTime - lastSample >= 8) {  // 125Hz sampling (8ms)
      lastSample = currentTime;
      
      if (ecgReady && ecgSensor.getECGSamples() > 0) {
        int32_t ecgValue = ecgSensor.getECGSample();
        
        ECGData data;
        data.timestamp = currentTime;
        data.value = ecgValue;
        
        ecgMutex.lock();
        ecgQueue.push(data);
        if (ecgQueue.size() > 100) ecgQueue.pop();
        ecgMutex.unlock();
        
        ecgLastUpdate = millis();
      }
    }
    
    threads.yield();
  }
}

// ========== DATA OUTPUT THREAD ==========
void outputThread() {
  uint32_t lastWatchdogCheck = 0;
  
  while (true) {
    // Watchdog check every second
    if (millis() - lastWatchdogCheck > 1000) {
      lastWatchdogCheck = millis();
      
      uint32_t currentTime = millis();
      sensorErrors = 0x00;
      
      // Check each sensor for timeout
      if (pcgReady && (currentTime - pcgLastUpdate > WATCHDOG_TIMEOUT_MS)) {
        sensorErrors |= ERROR_PCG;
      }
      if (accelReady && (currentTime - accelLastUpdate > WATCHDOG_TIMEOUT_MS)) {
        sensorErrors |= ERROR_ACCEL;
      }
      if (tempReady && (currentTime - tempLastUpdate > (WATCHDOG_TIMEOUT_MS + 5000))) {
        // Temp sensor has longer timeout due to 1Hz sampling
        sensorErrors |= ERROR_TEMP;
      }
      if (ecgReady && (currentTime - ecgLastUpdate > WATCHDOG_TIMEOUT_MS)) {
        sensorErrors |= ERROR_ECG;
      }
      
      // If any sensor failed, halt the entire system
      if (sensorErrors != 0x00) {
        systemHalted = true;
        
        Serial.println("\n========================================");
        Serial.println("!!! CRITICAL ERROR: SENSOR FAILURE !!!");
        Serial.println("========================================");
        Serial.print("Failed Sensors: ");
        if (sensorErrors & ERROR_PCG) Serial.print("PCG ");
        if (sensorErrors & ERROR_ACCEL) Serial.print("ACCEL ");
        if (sensorErrors & ERROR_TEMP) Serial.print("TEMP ");
        if (sensorErrors & ERROR_ECG) Serial.print("ECG ");
        Serial.println();
        Serial.println("System HALTED - Please restart the device");
        Serial.println("========================================\n");
        
        // Flash LED rapidly to indicate error
        while (true) {
          digitalWrite(LED_PIN, HIGH);
          delay(100);
          digitalWrite(LED_PIN, LOW);
          delay(100);
        }
      }
    }
    
    // Only output if system is not halted
    if (systemHalted) {
      threads.delay(1000);
      continue;
    }
    
    // Output PCG data
    pcgMutex.lock();
    if (!pcgQueue.empty()) {
      PCGData data = pcgQueue.front();
      pcgQueue.pop();
      pcgMutex.unlock();
      
      Serial.print("PCG,");
      Serial.print(data.timestamp);
      Serial.print(",");
      Serial.println(data.value);
    } else {
      pcgMutex.unlock();
    }
    
    // Output Accelerometer data
    accelMutex.lock();
    if (!accelQueue.empty()) {
      AccelData data = accelQueue.front();
      accelQueue.pop();
      accelMutex.unlock();
      
      Serial.print("ACCEL,");
      Serial.print(data.timestamp);
      Serial.print(",");
      Serial.print(data.x, 4);
      Serial.print(",");
      Serial.print(data.y, 4);
      Serial.print(",");
      Serial.println(data.z, 4);
    } else {
      accelMutex.unlock();
    }
    
    // Output Temperature data
    tempMutex.lock();
    if (!tempQueue.empty()) {
      TempData data = tempQueue.front();
      tempQueue.pop();
      tempMutex.unlock();
      
      Serial.print("TEMP,");
      Serial.print(data.timestamp);
      Serial.print(",");
      Serial.println(data.temperature, 2);
    } else {
      tempMutex.unlock();
    }
    
    // Output ECG data
    ecgMutex.lock();
    if (!ecgQueue.empty()) {
      ECGData data = ecgQueue.front();
      ecgQueue.pop();
      ecgMutex.unlock();
      
      Serial.print("ECG,");
      Serial.print(data.timestamp);
      Serial.print(",");
      Serial.println(data.value);
    } else {
      ecgMutex.unlock();
    }
    
    threads.yield();
  }
}

// ========== SENSOR VALIDATION ==========
bool validateSensors() {
  Serial.println("\n========== SENSOR VALIDATION ==========");
  sensorErrors = 0x00;
  
  // Validate PCG
  Serial.print("Validating PCG... ");
  uint16_t pcgTest = analogRead(PCG_PIN);
  if (pcgTest > 0 && pcgTest < PCG_ADC_MAX) {
    pcgReady = true;
    pcgLastUpdate = millis();
    Serial.println("OK");
  } else {
    sensorErrors |= ERROR_PCG;
    Serial.println("FAIL");
  }
  
  // Validate Accelerometer
  Serial.print("Validating LSM9DS1 Accelerometer... ");
  if (imu.accelAvailable()) {
    imu.readAccel();
    if (imu.ax != 0 || imu.ay != 0 || imu.az != 0) {
      accelReady = true;
      accelLastUpdate = millis();
      Serial.println("OK");
    } else {
      sensorErrors |= ERROR_ACCEL;
      Serial.println("FAIL - No data");
    }
  } else {
    sensorErrors |= ERROR_ACCEL;
    Serial.println("FAIL - Not available");
  }
  
  // Validate Temperature
  Serial.print("Validating MAX30205 Temperature... ");
  if (tempReady) {
    float testTemp = tempSensor.getTemperature();
    if (!isnan(testTemp) && testTemp > -40 && testTemp < 85) {
      tempLastUpdate = millis();
      Serial.print("OK (");
      Serial.print(testTemp, 2);
      Serial.println("°C)");
    } else {
      sensorErrors |= ERROR_TEMP;
      tempReady = false;
      Serial.println("FAIL - Invalid reading");
    }
  } else {
    sensorErrors |= ERROR_TEMP;
    Serial.println("FAIL - Not initialized");
  }
  
  // Validate ECG
  Serial.print("Validating MAX30003 ECG... ");
  if (ecgReady) {
    delay(100);
    if (ecgSensor.getECGSamples() > 0) {
      ecgLastUpdate = millis();
      Serial.println("OK");
    } else {
      sensorErrors |= ERROR_ECG;
      ecgReady = false;
      Serial.println("FAIL - No samples");
    }
  } else {
    sensorErrors |= ERROR_ECG;
    Serial.println("FAIL - Not initialized");
  }
  
  Serial.println("=======================================\n");
  
  if (sensorErrors == 0x00) {
    sensorsValidated = true;
    Serial.println("✓ All sensors validated successfully!");
    return true;
  } else {
    Serial.println("✗ Sensor validation failed. Check connections.");
    Serial.println("System will NOT start until all sensors are operational.");
    return false;
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.println("\n========================================");
  Serial.println("Multi-Sensor Data Acquisition System");
  Serial.println("Teensy 4.1 - Medical Sensor Array");
  Serial.println("========================================\n");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("I2C initialized at 400kHz");
  
  // Initialize SPI for ECG
  SPI.begin();
  pinMode(ECG_CS_PIN, OUTPUT);
  digitalWrite(ECG_CS_PIN, HIGH);
  Serial.println("SPI initialized for ECG");
  
  // Initialize Temperature Sensor
  Serial.println("\nInitializing MAX30205 Temperature Sensor...");
  tempSensor.begin();
  delay(100);
  if (tempSensor.scanAvailableSensors()) {
    tempReady = true;
    Serial.println("✓ Temperature sensor initialized");
  } else {
    Serial.println("✗ Temperature sensor not found");
  }
  
  // Initialize Accelerometer
  Serial.println("\nInitializing LSM9DS1 Accelerometer...");
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.accel.scale = 2;
  imu.settings.accel.sampleRate = 6;
  imu.settings.accel.bandwidth = 2;
  imu.settings.accel.enableX = true;
  imu.settings.accel.enableY = true;
  imu.settings.accel.enableZ = true;
  
  if (imu.begin()) {
    accelReady = true;
    Serial.println("✓ Accelerometer initialized");
  } else {
    Serial.println("✗ Accelerometer initialization failed");
  }
  
  // Initialize ECG Sensor
  Serial.println("\nInitializing MAX30003 ECG Sensor...");
  pinMode(ECG_INT_PIN, INPUT_PULLUP);
  ecgSensor.max30003Begin(ECG_CS_PIN);
  delay(100);
  
  if (ecgSensor.max30003ReadInfo()) {
    ecgSensor.max30003BeginECG();
    ecgReady = true;
    Serial.println("✓ ECG sensor initialized");
  } else {
    Serial.println("✗ ECG sensor initialization failed");
  }
  
  // Start Master Timer
  masterTimer.begin(masterTimerISR, MASTER_PERIOD_US);
  Serial.println("\n✓ Master timer started at 1kHz");
  
  // Validate all sensors
  delay(500);
  validateSensors();
  
  if (sensorsValidated) {
    // Start all threads
    Serial.println("\nStarting acquisition threads...");
    threads.addThread(pcgThread);
    threads.addThread(accelThread);
    threads.addThread(tempThread);
    threads.addThread(ecgThread);
    threads.addThread(outputThread);
    
    Serial.println("\n✓✓✓ ALL SYSTEMS OPERATIONAL ✓✓✓");
    Serial.println("Data format: SENSOR,TIMESTAMP,VALUE(S)");
    Serial.println("Watchdog monitoring active - system will halt on sensor failure\n");
  } else {
    Serial.println("\n✗✗✗ SYSTEM CANNOT START - SENSOR ERRORS ✗✗✗");
    Serial.println("Please check connections and restart the device.\n");
    
    // Flash LED slowly to indicate initialization failure
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
  
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Blink LED to show system is alive (only if not halted)
  if (!systemHalted) {
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink > 1000) {
      lastBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}