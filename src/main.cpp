#include <Wire.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <BMP085.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <Servo.h>

// ===== PIN CONFIGURATION =====
#define LED_PIN                 PC13    // Onboard LED
#define MPU_INT_PIN             PA0     // Interrupt pin for MPU6050
#define ESC_PIN_1               PA8
#define ESC_PIN_2               PA9

// ===== CONFIGURATION =====
#define SERIAL_SPEED            115200  // Serial communication speed
#define PRINT_INTERVAL          10      // How often to print data (ms)
#define PRINT_QUATERNION        0       // Enable / disable print quaternion
#define PRINT_GRAVITY           0       // Enable / disable print gravity
#define PRINT_YAW_PITCH_ROLL    0       // Enable / disable print ypr
#define PRINT_BMP_DATA          0       // Enable / disable print BMP
#define PRINT_CORRECTION        1       // Enable / disable print PID control correction
#define OVERFLOW_LED_ON_TIME    100     // LED on time in ms
#define OVERFLOW_LED_OFF_TIME   100     // LED off time in ms
#define OVERFLOW_BLINK_COUNT    5       // Number of blinks for overflow indicator
#define CALIBRATE               0       // Calibaration 0 FALSE 1 TRUE
#define ENABLE_BMP              0       // Turn on / off BMP
#define ESC_ARMING_TIME         3000    // Time to wait for ESC to arm
#define ESC_RECOVERY_TIMEOUT    500     // Interval for ESC recovery
#define ESC_MAX_ERROR           5       // Error count before locking up
#define MOTOR_BASE_SPEED        1200    // Base speed for motor to run (min 1200 to spin)
#define MOTOR_MAX_SPEED         1700    // Max speed for motor (can go up to 2200)

// ===== SENSOR INSTANCES =====
MPU6050 mpu;
BMP085 bmp;
Servo esc1;
Servo esc2;

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
float pitchRate = 0.0f;
float rollRate = 0.0f;
float gyroScale;

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

ESCStatus escStatus[2];
ESCState escSystemState = ESC_INIT;
unsigned long escLastTime = 0;
int actualEsc1Speed = 0;
int actualEsc2Speed = 0;

// ===== CONTROL VARIABLES =====
Quaternion qTarget;
VectorFloat qAxis;
float qAngle;
float lastUpdateControlTime = 0.0f;

// ===== PID VARIABLES =====
// ----- OUTER LOOP -----
float p_angleValue = 1.5;
float i_angleValue = 0.1f;
float i_angleMax = 10.0f;
float d_angleValue = 0.0f;
float I_pitchAngleError = 0.0f;
float previousPitchAngleError = 0.0f;
// ----- INNER LOOP -----
float p_rateValue = 2.0f;
float i_rateValue = 0.3f;
float i_rateMax = 50.0f;
float d_rateValue = 0.08f;
float d_rateAlpha = 0.5;
float backCalculationGain = 0.2f;
float I_pitchRateError = 0.0f;
float previousPitchRateError = 0.0f;

// ===== SERIAL VARIABLES =====
char cmdBuffer[4];
int cmdIndex = 0;
bool cmdReady = false;
bool enableESC = false;

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
void quaternionToAxisAngle(const Quaternion &quat, VectorFloat &axis, float &angle);
void applyControl();
void checkSerial();
void processCommand();
void readGyro();

// ===== SERIAL COM =====
void checkSerial() {
  while (Serial.available() > 0 && !cmdReady) {
    char inChar = (char)Serial.read();
    
    // Process on newline
    if (inChar == '\n' || inChar == '\r') {
      if (cmdIndex > 0) {
        cmdReady = true;
      }
    } else if (cmdIndex < 4 - 1) {
      cmdBuffer[cmdIndex++] = inChar;
    }
  }
}

void processCommand() {
  if (!cmdReady) return;

  cmdBuffer[cmdIndex] = '\0';

  char command = cmdBuffer[0];
  
  switch (command) {
    case 'P':
      break;
    case 'p':
      break;
    case 'I':
      break;
    case 'i':
      break;
    case 'D':
      break;
    case 'd':
      break;
    case 'S':
      break;
    case 's':
      break;
    case 'K':
    case 'k':
      enableESC = !enableESC;
      break;
    default:
      Serial.println("WRONG COMMAND: [P/I/D/S] to increase [p/i/d/s] to decrease");
      break;
  }

  cmdIndex = 0;
  cmdReady = false;
}

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

// ===== CONTROL SYSTEM =====
void quaternionToAxisAngle(const Quaternion &quat, VectorFloat &axis, float &angle) {
  angle = 2.0f * acos(quat.w);
  float s = sqrt(1.0f - quat.w * quat.w);
  if (s < 0.0001f) {
    axis.x = 1.0f;
    axis.y = 0.0f;
    axis.z = 0.0f;
  } else {
    axis.x = quat.x / s;
    axis.y = quat.y / s;
    axis.z = quat.z / s;
  }
}

void readGyro() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float _pitchRate = (float)gy / gyroScale;

  // Apply low pass filter
  static float filteredPitchRateState = 0.0f;
  filteredPitchRateState = 0.7 * filteredPitchRateState + 0.3 * _pitchRate;
  pitchRate = filteredPitchRateState;
}

void applyControl() {
  float currentTime = millis() / 1000.0f;
  float dt = currentTime - lastUpdateControlTime;
  lastUpdateControlTime = currentTime;

  if (dt > 0.1f || dt <= 0.0f) {
    dt = 0.01f;
  }

  // Get latest gyro rate
  readGyro();

  // ---- OUTER LOOP ----
  // Quaternion process to get rotaion difference from target
  Quaternion conjugate = q.getConjugate();
  conjugate.normalize();
  Quaternion qError = qTarget.getProduct(conjugate);

  // Convert to X, Y, Z axis and get the angle
  quaternionToAxisAngle(qError, qAxis, qAngle);

  // Calculate the error
  float pitchAngleError = qAxis.y * qAngle;

  // P Angle Term
  float P_angle = pitchAngleError * p_angleValue;

  // I Angle Term
  I_pitchAngleError += pitchAngleError * i_angleValue * dt;
  I_pitchAngleError = constrain(I_pitchAngleError, -i_angleMax, i_angleMax);
  float I_angle = I_pitchAngleError;

  // D Angle Term
  // Often not used because we calcuated rate of change using rate velocity in inner loop
  float D_angle = 0.0f;
  if (d_angleValue > 0.0f) {
    D_angle = (pitchAngleError - previousPitchAngleError) / dt;
    static float filteredD_angle = 0.0f;
    filteredD_angle = 0.5 * filteredD_angle + 0.5 * D_angle;
    D_angle = filteredD_angle * d_angleValue;
  }
  previousPitchAngleError = pitchAngleError;

  float targetPitchRate = P_angle + I_angle + D_angle;

  // ----- INNER LOOP -----
  float pitchRateError = targetPitchRate - pitchRate;

  // P Rate Term
  float P_rate = pitchRateError * p_rateValue;

  // I Rate Term
  float I_rateContribution = pitchRateError * dt * i_rateValue;
  float I_potentialRate = I_pitchRateError + I_rateContribution;
  I_potentialRate = constrain(I_potentialRate, -i_rateMax, i_rateMax);
  float I_rate = I_potentialRate;
  
  // D Rate Term
  float D_rate = (pitchRateError - previousPitchRateError) / dt;
  static float filteredD_rate = 0.0f;
  filteredD_rate = d_rateAlpha * filteredD_rate + (1.0f - d_rateAlpha) * D_rate;
  D_rate = filteredD_rate * d_rateValue;

  previousPitchRateError = pitchRateError;
  
  // Desired correction for motor (before saturation)
  float desiredCorrection = P_rate + I_rate + D_rate;
  int desiredEsc1Speed = MOTOR_BASE_SPEED + desiredCorrection;
  int desiredEsc2Speed = MOTOR_BASE_SPEED - desiredCorrection;

  actualEsc1Speed = constrain(desiredEsc1Speed, MOTOR_BASE_SPEED, MOTOR_MAX_SPEED);
  actualEsc2Speed = constrain(desiredEsc2Speed, MOTOR_BASE_SPEED, MOTOR_MAX_SPEED);

  // Anti-Windup for I Rate Term
  // Check if its both of some of the speed is exceeding
  float correctionSaturation = (actualEsc1Speed - actualEsc2Speed) - (desiredEsc1Speed - desiredEsc2Speed); 
  if (abs(correctionSaturation) > 0.1f) {
    float maxControlRange = 2.0 * (MOTOR_BASE_SPEED - MOTOR_MAX_SPEED);
    if (maxControlRange == 0.0f) maxControlRange = 1.0f; // Avoid division by zero

    // Get adjustment value (???)
    float adjustment = (I_rateContribution * backCalculationGain * abs(correctionSaturation)) / maxControlRange;
    I_pitchRateError += I_rateContribution - adjustment;

    // Re-clamp the pitch error
    I_pitchRateError = constrain(I_pitchRateError, -i_rateMax, i_rateMax);
  } else {
    I_pitchRateError = I_potentialRate;
  }

  esc1.writeMicroseconds(actualEsc1Speed);
  esc2.writeMicroseconds(actualEsc2Speed);

  #if PRINT_CORRECTION
  float _currentTime = millis();
  if (_currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = _currentTime;
    Serial.print("PitchAngErr: "); Serial.print(pitchAngleError); Serial.print(" | ");
    Serial.print("TargetRate: "); Serial.print(targetPitchRate); Serial.print(" | ");
    Serial.print("PitchRate: "); Serial.print(pitchRate); Serial.print(" | ");
    Serial.print("PitchRateErr: "); Serial.print(pitchRateError); Serial.print(" | ");
    // Serial.print("P_rate: "); Serial.print(P_rate); Serial.print(" | ");
    Serial.print("I_rate: "); Serial.print(I_rate); Serial.print(" | ");
    // Serial.print("I_raw: "); Serial.print(I_pitchRateError); Serial.print(" | "); // Raw integrator state
    // Serial.print("D_rate: "); Serial.print(D_rate); Serial.print(" | ");
    Serial.print("DesiredCorr: "); Serial.print(desiredCorrection); Serial.print(" | ");
    Serial.print("ESC1: "); Serial.print(actualEsc1Speed); Serial.print(" | ");
    Serial.print("ESC2: "); Serial.print(actualEsc2Speed);
    Serial.println();
  }
  #endif
}

// ===== NON-BLOCK ESC SETUP AND UPDATE =====
void initializeESCs() {
  escStatus[0] = {ESC_INIT, 0, false, ESC_PIN_1, 0};
  escStatus[1] = {ESC_INIT, 0, false, ESC_PIN_2, 0};

  escSystemState = ESC_INIT;
  escLastTime = millis();
}

void setupESCs() {
  esc1.attach(escStatus[0].pin);
  esc2.attach(escStatus[1].pin);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);

  for (size_t i = 0; i < 2; i++) {
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

  for (size_t i = 0; i < 2; i++) {
    if (!escStatus[i].connected) {
      allConnected = false;
    }

    if (escStatus[i].state == ESC_RECOVERY) {
      anyRecovery = true;

      switch (i) {
        case 0:
          esc1.attach(escStatus[i].pin);
          esc1.writeMicroseconds(MOTOR_BASE_SPEED);
          break;
        case 1:
          esc1.attach(escStatus[i].pin);
          esc1.writeMicroseconds(MOTOR_BASE_SPEED);
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
  esc2.writeMicroseconds(1000);
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
        for (size_t i = 0; i < 2; i++) {
          escStatus[i].state = ESC_READY;
          escStatus[i].errorCount = 0;
          escStatus[i].responseLastTime = millis();
        }
      }
      break;
    case ESC_READY:
      if (!dmpReady) return;
      if (enableESC) {
        applyControl();
      } else {
        shutdownESCs();
      }
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
  q.normalize();

  #if PRINT_GRAVITY || PRINT_YAW_PITCH_ROLL
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  #endif
}

// ===== DATA PRINTING =====
void printData() {
  unsigned long currentMillis = millis();
  
  // Print at regular intervals
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentMillis;

    #if PRINT_QUATERNION
    Serial.print("QUATERNION \t");
    Serial.print("W "); Serial.print(q.w); Serial.print("\t");
    Serial.print("X "); Serial.print(q.x); Serial.print("\t");
    Serial.print("Y "); Serial.print(q.y); Serial.print("\t");
    Serial.print("Z "); Serial.print(q.z); Serial.print("\t | ");
    #endif

    #if PRINT_GRAVITY
    Serial.print("GRAVITY \t");
    Serial.print("X "); Serial.print(gravity.x); Serial.print("\t");
    Serial.print("Y "); Serial.print(gravity.y); Serial.print("\t");
    Serial.print("Z "); Serial.print(gravity.z); Serial.print("\t | ");
    #endif

    #if PRINT_YAW_PITCH_ROLL
    Serial.print("YAW PITCH ROLL \t");
    Serial.print("YAW "); Serial.print(ypr[0] * RAD_TO_DEG); Serial.print("\t");
    Serial.print("PITCH "); Serial.print(ypr[1] * RAD_TO_DEG); Serial.print("\t");
    Serial.print("ROLL "); Serial.print(ypr[2] * RAD_TO_DEG); Serial.print("\t | ");
    #endif

    #if PRINT_BMP_DATA
    Serial.print("Temp: ");
    Serial.print(temperature, 1);
    Serial.print("°C Pressure: ");
    Serial.print(pressure/100.0, 1); // Convert to hPa
    Serial.print("hPa Alt: ");
    Serial.print(altitude);
    Serial.print("m");
    #endif

    #if PRINT_QUATERNION || PRINT_YAW_PITCH_ROLL || PRINT_GRAVITY || PRINT_BMP_DATA
    Serial.print("\n");
    #endif
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

  // Set MPU gyro scale
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  gyroScale = 131.0;
  
  #if ENABLE_BMP  // Initialize BMP085
  bmp.initialize();
  if (!bmp.testConnection()) {
    errorBlink(3);
  }
  #endif
  
  // Initialize DMP
  devStatus = mpu.dmpInitialize();

  // Initialize ESCs
  initializeESCs();

  #if !CALIBRATE
  mpu.setXGyroOffset(2);
  mpu.setYGyroOffset(65);
  mpu.setZGyroOffset(-38);
  mpu.setXAccelOffset(826);
  mpu.setYAccelOffset(-1701);
  mpu.setZAccelOffset(968);
  #endif
  
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

  lastUpdateControlTime = millis() / 1000.0f;
}

// ===== MAIN LOOP =====
void loop() {
  // Skip if DMP not ready
  if (!dmpReady) return;

  if (overflowBlinkCount > 0) {
    overflowBlink();
  }

  checkSerial();
  processCommand();
  
  #if ENABLE_BMP
  // Update BMP readings (non-blocking)
  updateBMP();
  #endif

  // Update ESC (non-blocking)
  updateESCs();

  // Process MPU data if available
  if (mpuInterrupt || fifoCount >= packetSize) {
    processMPU();
  }
  
  // Print data at regular intervals
  printData();
}