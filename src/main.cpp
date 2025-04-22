#include <Wire.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <BMP085.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ===== CONFIGURATION =====
#define LED_PIN         PC13    // Onboard LED
#define MPU_INT_PIN     PA0     // Interrupt pin for MPU6050
#define SERIAL_SPEED    115200  // Serial communication speed
#define PRINT_INTERVAL  10     // How often to print data (ms)
#define OVERFLOW_LED_ON_TIME 100 // LED on time in ms
#define OVERFLOW_LED_OFF_TIME 100 // LED off time in ms
#define OVERFLOW_BLINK_COUNT 5 // Number of blinks for overflow indicator

// ===== SENSOR INSTANCES =====
MPU6050 mpu;
BMP085 bmp;

// ===== MPU6050 VARIABLES =====
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
Quaternion rotation;
Quaternion rotated;
VectorFloat gravity;
volatile bool mpuInterrupt = false;
unsigned long lastPrintTime = 0;

// ===== LED VARIABLES =====
unsigned long ledBlinkTime = 0;    // Timestamp for LED state changes
bool ledState = false;             // Current LED state (on/off)
uint8_t overflowBlinkCount = 0;    // Number of blinks remaining for overflow indication
uint32_t fifoOverflowCount = 0;    // Total count of FIFO overflows

// ===== BMP085 VARIABLES =====
enum BMP085State {
  BMP_IDLE,
  BMP_TEMP_REQUESTED,
  BMP_PRESSURE_REQUESTED
};

BMP085State bmpState = BMP_IDLE;
unsigned long bmpLastTime = 0;
const unsigned long BMP_TEMP_DELAY = 5;        // ms for temperature reading
const unsigned long BMP_PRESSURE_DELAY = 26;   // ms for pressure reading
const unsigned long BMP_UPDATE_INTERVAL = 1000; // How often to update BMP readings (ms)
float temperature = 0;
float pressure = 0;
int32_t altitude = 0;

// ===== FUNCTION PROTOTYPES =====
void dmpDataReady();
void errorBlink(uint8_t blinks);
String getOrientation(const Quaternion& q);
void updateBMP();
void processMPU();
void printData();
void overflowBlink();

// ===== INTERRUPT HANDLER =====
void dmpDataReady() {
  mpuInterrupt = true;
}

// ===== ERROR INDICATOR =====
void errorBlink(uint8_t blinks) {
  pinMode(LED_PIN, OUTPUT);
  
  while (true) {
    for (uint8_t i = 0; i < blinks; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
    delay(1000);
  }
}

void overflowBlink() {
  unsigned long currentMillis = millis();

  if (ledState && (currentMillis - ledBlinkTime >= OVERFLOW_LED_ON_TIME)) {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
    ledBlinkTime = currentMillis;
    overflowBlinkCount--;
  } else if (!ledState && (currentMillis - ledBlinkTime >= OVERFLOW_LED_OFF_TIME)) {
    digitalWrite(LED_PIN, HIGH);
    ledState = true;
    ledBlinkTime = currentMillis;
    overflowBlinkCount--;
  }
}

// ===== ORIENTATION DETECTION =====
String getOrientation(const Quaternion& q) {
  // Calculate gravity vector from quaternion
  float gx = 2 * (q.x * q.z - q.w * q.y);
  float gy = 2 * (q.w * q.x + q.y * q.z);
  float gz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
  
  // Find dominant axis
  float absX = abs(gx);
  float absY = abs(gy);
  float absZ = abs(gz);
  
  if (absZ > absX && absZ > absY) {
    return gz > 0 ? "Flat" : "Upside down";
  } else if (absY > absX) {
    return gy > 0 ? "Left side up" : "Right side up";
  } else {
    return gx > 0 ? "Front side up" : "Back side up";
  }
}

// ===== NON-BLOCKING BMP085 UPDATE =====
void updateBMP() {
  unsigned long currentMillis = millis();
  
  switch(bmpState) {
    case BMP_IDLE:
      if (currentMillis - bmpLastTime >= BMP_UPDATE_INTERVAL) {
        bmpState = BMP_TEMP_REQUESTED;
        bmp.setControl(BMP085_MODE_TEMPERATURE);
        bmpLastTime = currentMillis;
      }
      break;
      
    case BMP_TEMP_REQUESTED:
      if (currentMillis - bmpLastTime >= BMP_TEMP_DELAY) {
        temperature = bmp.getTemperatureC();
        bmp.setControl(BMP085_MODE_PRESSURE_3);
        bmpState = BMP_PRESSURE_REQUESTED;
        bmpLastTime = currentMillis;
      }
      break;
      
    case BMP_PRESSURE_REQUESTED:
      if (currentMillis - bmpLastTime >= BMP_PRESSURE_DELAY) {
        pressure = bmp.getPressure();
        altitude = bmp.getAltitude(pressure);
        bmpState = BMP_IDLE;
        bmpLastTime = currentMillis;
      }
      break;
  }
}

// ===== MPU DATA PROCESSING =====
void processMPU() {
  // Reset interrupt flag
  mpuInterrupt = false;
  
  // Get MPU status
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  // Handle overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    fifoOverflowCount++;

    if (overflowBlinkCount == 0) {
      overflowBlinkCount = OVERFLOW_BLINK_COUNT * 2;
      ledState = true;
      digitalWrite(LED_PIN, HIGH);
      ledBlinkTime = millis();
    }

    return;
  }
  
  // Check for DMP data ready
  if (!(mpuIntStatus & 0x02)) return;
  
  // Wait for complete packet
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
  // Read packet
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  
  // Get quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // Apply rotation (quaternion multiplication)
  rotated.w = q.w * rotation.w - q.x * rotation.x - q.y * rotation.y - q.z * rotation.z;
  rotated.x = q.w * rotation.x + q.x * rotation.w + q.y * rotation.z - q.z * rotation.y;
  rotated.y = q.w * rotation.y - q.x * rotation.z + q.y * rotation.w + q.z * rotation.x;
  rotated.z = q.w * rotation.z + q.x * rotation.y - q.y * rotation.x + q.z * rotation.w;

  q = rotated;
}

// ===== DATA PRINTING =====
void printData() {
  unsigned long currentMillis = millis();
  
  // Print at regular intervals
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentMillis;
    // Print quaternion values
    Serial.print("quat\t");
    Serial.print(q.w); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t| ");
        
    // Print orientation using the rotated quaternion
    Serial.print("Orientation: ");
    Serial.print(getOrientation(rotated));
    
    // Print MPU data
    Serial.print("Orientation: ");
    Serial.print(getOrientation(q));

    Serial.print(" | Temp: ");
    Serial.print(temperature, 1);
    Serial.print("°C | Pressure: ");
    Serial.print(pressure/100.0, 1); // Convert to hPa
    Serial.print("hPa | Alt: ");
    Serial.print(altitude);
    Serial.println("m");
  }
}

// ===== SETUP =====
void setup() {
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize communication
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(SERIAL_SPEED);
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    errorBlink(2);
  }
  
  // Initialize BMP085
  bmp.initialize();
  if (!bmp.testConnection()) {
    errorBlink(3);
  }
  
  // Initialize DMP
  devStatus = mpu.dmpInitialize();
  
  if (devStatus == 0) {
    // Enable DMP
    mpu.setDMPEnabled(true);
    
    // Enable interrupt detection
    attachInterrupt(MPU_INT_PIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // Set DMP ready flag
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    rotation.w = cos(PI/4); 
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = sin(PI/4); 
  } else {
    // ERROR!
    errorBlink(4);
  }
}

// ===== MAIN LOOP =====
void loop() {
  // Skip if DMP not ready
  if (!dmpReady) return;

  if (overflowBlinkCount > 0) {
    overflowBlink();
  }
  
  // Update BMP readings (non-blocking)
  updateBMP();

  // Process MPU data if available
  if (mpuInterrupt || fifoCount >= packetSize) {
    processMPU();
  }
  
  // Print data at regular intervals
  printData();
}