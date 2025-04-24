#include <Wire.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <BMP085.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>

// ===== PIN CONFIGURATION =====
#define LED_PIN                 PC13    // Onboard LED
#define MPU_INT_PIN             PA0     // Interrupt pin for MPU6050
#define ESC_PIN_1               PA8

// ===== CONFIGURATION =====
#define SERIAL_SPEED            115200  // Serial communication speed
#define PRINT_INTERVAL          10      // How often to print data (ms)
#define OVERFLOW_LED_ON_TIME    100     // LED on time in ms
#define OVERFLOW_LED_OFF_TIME   100     // LED off time in ms
#define OVERFLOW_BLINK_COUNT    5       // Number of blinks for overflow indicator
#define CALIBRATE               0       // Calibaration 0 FALSE 1 TRUE
#define ESC_ARMING_TIME         3000    // Time to wait for ESC to arm
#define ESC_RECOVERY_TIMEOUT    500     // Interval for ESC recovery
#define ESC_MAX_ERROR           5       // Error count before locking up

// ===== SENSOR INSTANCES =====
MPU6050 mpu;
BMP085 bmp;
Servo esc1;

// ===== MPU6050 VARIABLES =====
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
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

// ===== ESC VARIABLES =====
enum ESCState {
  ESC_INIT,
  ESC_ARMING,
  ESC_READY,
  ESC_ERROR,
  ESC_RECOVERY
};

struct ESCStatus {
  ESCState state;
  unsigned long responseLastTime;
  bool connected;
  uint8_t pin;
  uint8_t errorCount;
};

ESCStatus escStatus[1];
ESCState escSystemState = ESC_INIT;
unsigned long escLastTime = 0;

// ===== FUNCTION PROTOTYPES =====
void dmpDataReady();
void errorBlink(uint8_t blinks);
void updateBMP();
void processMPU();
void printData();
void overflowBlink();
void initializeESCs();
void setupESCs();
void checkESCsConnection();
void recoverESCs();
void shutdownESCs();

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

// ===== NON-BLOCK ESC SETUP AND UPDATE
void initializeESCs() {
  escStatus[0] = {ESC_INIT, 0, false, ESC_PIN_1, 0};

  escSystemState = ESC_INIT;
  escLastTime = millis();
}

void setupESCs() {
  esc1.attach(escStatus[0].pin);

  esc1.writeMicroseconds(1000);

  for (size_t i = 0; i < 1; i++) {
    escStatus[i].responseLastTime = millis();
    escStatus[i].connected = true;
    escStatus[i].state = ESC_ARMING;
  }

  escSystemState = ESC_ARMING;
  escLastTime = millis();
}

void checkESCsConnection() {
  // TODO: NEED SEPERATE HARDWARE TO CHECK CONNECTION FOR PWM
}

void recoverESCs() {
  bool allConnected = true;
  bool anyRecovery = false;

  for (size_t i = 0; i < 1; i++) {
    if (!escStatus[i].connected) {
      allConnected = false;
    }

    if (escStatus[i].state == ESC_RECOVERY) {
      anyRecovery = true;

      switch (i) {
        case 0:
          esc1.attach(escStatus[i].pin);
          esc1.writeMicroseconds(1000);
          break;
      }
    }

    if (escStatus[i].errorCount >= ESC_MAX_ERROR) {
      escStatus[i].state = ESC_ERROR;
    } else {
      escStatus[i].state = ESC_ARMING;
    }
  }

  if (allConnected && !anyRecovery) {
    escSystemState = ESC_ARMING;
    escLastTime = millis();
  }
}

void shutdownESCs() {
  esc1.writeMicroseconds(1000);
}

void updateESCs() {
  unsigned long currentMillis = millis();

  checkESCsConnection();

  switch (escSystemState) {
    case ESC_INIT:
      setupESCs();
      break;
    case ESC_ARMING:
      if (currentMillis - escLastTime >= ESC_ARMING_TIME) {
        escSystemState = ESC_READY;
        escLastTime = currentMillis;
        for (size_t i = 0; i < 1; i++) {
          escStatus[i].state = ESC_READY;
          escStatus[i].errorCount = 0;
          escStatus[i].responseLastTime = millis();
        }
      }
      break;
    case ESC_READY:
      if (!dmpReady) return;
      esc1.writeMicroseconds(1200);
      break;
    case ESC_RECOVERY:
      recoverESCs();
      // Timeout if recovery takes too long
      if (currentMillis - escLastTime >= ESC_RECOVERY_TIMEOUT) {
        escSystemState = ESC_ERROR;
        escLastTime = currentMillis;
      }
      break;
    case ESC_ERROR:
      shutdownESCs();

      if (currentMillis - escLastTime >= ESC_RECOVERY_TIMEOUT) {
        escSystemState = ESC_RECOVERY;
        escLastTime = currentMillis;
      }
      break;
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

    // Important: Re-enable DMP after overflow to ensure we get new interrupts
    mpu.setDMPEnabled(false);
    delay(5);  // Short delay to ensure settings take effect
    mpu.resetFIFO();  // Reset FIFO again to ensure clean state
    mpu.setDMPEnabled(true);

    return;
  }
  
  // Check for DMP data ready
  if (!(mpuIntStatus & 0x02)) return;
  
  // Wait for complete packet
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
  // Read packet
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  
  // Get quaternion, gravity and Yaw Pitch Roll
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

// ===== DATA PRINTING =====
void printData() {
  unsigned long currentMillis = millis();
  
  // Print at regular intervals
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentMillis;

    // Print quaternion values
    Serial.print("QUATERNION \t");
    Serial.print("W "); Serial.print(q.w); Serial.print("\t");
    Serial.print("X "); Serial.print(q.x); Serial.print("\t");
    Serial.print("Y "); Serial.print(q.y); Serial.print("\t");
    Serial.print("Z "); Serial.print(q.z); Serial.print("\n ");
    
    // Serial.print("GRAVITY \t");
    // Serial.print("X "); Serial.print(gravity.x); Serial.print("\t");
    // Serial.print("Y "); Serial.print(gravity.y); Serial.print("\t");
    // Serial.print("Z "); Serial.print(gravity.z); Serial.print("\r");

    // Serial.print("YAW PITCH ROLL \t");
    // Serial.print("YAW "); Serial.print(ypr[0] * RAD_TO_DEG); Serial.print("\t");
    // Serial.print("PITCH "); Serial.print(ypr[1] * RAD_TO_DEG); Serial.print("\t");
    // Serial.print("ROLL "); Serial.print(ypr[2] * RAD_TO_DEG); Serial.print("\r");

    // Print BMP data
    // Serial.print(" | Temp: ");
    // Serial.print(temperature, 1);
    // Serial.print("°C | Pressure: ");
    // Serial.print(pressure/100.0, 1); // Convert to hPa
    // Serial.print("hPa | Alt: ");
    // Serial.print(altitude);
    // Serial.println("m");
  }
}

// ===== SETUP =====
void setup() {
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize communication
  Wire.begin();
  Wire.setClock(100000);
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

  // Initialize ESCs
  initializeESCs();

  mpu.setXGyroOffset(1);
  mpu.setYGyroOffset(64);
  mpu.setZGyroOffset(-31);
  mpu.setXAccelOffset(718);
  mpu.setYAccelOffset(-1734);
  mpu.setZAccelOffset(972);
  
  if (devStatus == 0) {
    #if CALIBRATE
      mpu.CalibrateAccel(7);
      mpu.CalibrateGyro(7);

      Serial.println("Calibration complete!");
      Serial.print("Accel offsets: ");
      Serial.print(mpu.getXAccelOffset()); Serial.print(", ");
      Serial.print(mpu.getYAccelOffset()); Serial.print(", ");
      Serial.println(mpu.getZAccelOffset());
      
      Serial.print("Gyro offsets: ");
      Serial.print(mpu.getXGyroOffset()); Serial.print(", ");
      Serial.print(mpu.getYGyroOffset()); Serial.print(", ");
      Serial.println(mpu.getZGyroOffset());
    #endif

    mpu.setDMPEnabled(true);
    
    // Enable interrupt detection
    attachInterrupt(MPU_INT_PIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // Set DMP ready flag
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
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

  // Update ESC (non-blocking)
  updateESCs();

  // Process MPU data if available
  if (mpuInterrupt || fifoCount >= packetSize) {
    processMPU();
  }
  
  // Print data at regular intervals
  printData();
}