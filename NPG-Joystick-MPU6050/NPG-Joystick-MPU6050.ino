// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech
// Copyright (c) 2026 Varun Patil - vap05072006@gmail.com

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

// Core includes
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <BleCombo.h>

// ── MPU6050 Head Movement Includes ──
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ══════════════════════════════════════════════════════════════════════════════
// ─── CONTROL MAPPING CONFIGURATION ───
// ══════════════════════════════════════════════════════════════════════════════
// Head movement -> Mouse cursor movement
// Double blink -> Left mouse click
// Triple blink -> Right mouse click

// ══════════════════════════════════════════════════════════════════════════════
// ─── EASY-TO-ADJUST MOUSE CONTROL SETTINGS ───
// ══════════════════════════════════════════════════════════════════════════════

//  BASIC SETTINGS (ADJUST THESE TO FINE-TUNE)
#define MOUSE_UPDATE_RATE 12 // Update frequency: LOWER = faster updates (8-20)
#define DEADZONE 4.0       // Rest zone: HIGHER = easier to stop (0.3-2.0)
#define MIN_SENSITIVITY 0.15 // Slowest speed: LOWER = more precise (0.1-0.5)
#define MAX_SENSITIVITY 8.0  // Fastest speed: LOWER = more controlled (4.0-15.0)

//  CALIBRATION SETTINGS
#define GYRO_BIAS_SAMPLES 200  // samples averaged for bias at rest

//  SMOOTHING SETTINGS (FOR RESPONSIVENESS)
#define MOVEMENT_SMOOTHING 0.70 // Movement filter: LOWER = more responsive (0.5-0.85)
#define VELOCITY_DECAY 0.80     // Stop speed: LOWER = stops faster (0.7-0.9)
#define STOP_THRESHOLD 0.2      // Complete stop point: LOWER = stops sooner (0.1-0.5)

//  ACCELERATION SETTINGS
#define ACCEL_CURVE 2.5      // Acceleration curve: HIGHER = faster acceleration (1.5-4.0)
#define ACCEL_MULTIPLIER 2.8 // Acceleration strength: HIGHER = more acceleration (2.0-4.0)

//  RANGE SETTINGS
#define MAX_TILT_ANGLE 20.0 // Maximum head tilt: LOWER = shorter range (15.0-30.0)

// ══════════════════════════════════════════════════════════════════════════════

// ── VIBRATION MOTOR PIN ──
#define VIBRATION_PIN 7 // Vibration motor for calibration feedback
#define PIN_NEOPIXEL 15
#define BATTERY_VOLTAGE_PIN A6
#define BLUE_LED_DURATION 100

Adafruit_NeoPixel pixel(6, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5
#define IMU_LED 3

// ─── MPU6050 Head Movement Variables ───
Adafruit_MPU6050 mpu;
uint32_t mpuAddress = 0x68;

// Mouse control variables
float neutralPitch = 0;
float neutralRoll = 0;
float mouseVelocityX = 0, mouseVelocityY = 0;
bool isMPUCalibrated = false;
unsigned long lastMouseUpdate = 0;
bool axisCalibrated = false;

// ── NON-BLOCKING CALIBRATION STATE MACHINE ──
enum CalibrationState
{
  CAL_IDLE,
  CAL_INIT_WAIT,
  CAL_UP_VIBRATE,
  CAL_UP_WAIT,
  CAL_LEFT_VIBRATE,
  CAL_LEFT_WAIT,
  CAL_NEUTRAL_SAMPLE,
  CAL_COMPLETE
};

CalibrationState calState = CAL_IDLE;
unsigned long calStateStartTime = 0;

// ── GYRO BIAS (subtracted from every reading) ──
float gyroBias[3] = { 0, 0, 0 };

// ── LEARNED AXIS MAPPING ──
int xAxisIndex = 0;
int xAxisSign = 1;
int yAxisIndex = 1;
int yAxisSign = 1;

// ── GESTURE ACCUMULATION (during calibration) ──
float gestureSum[3] = { 0, 0, 0 };
unsigned long lastGyroMicros = 0;

// ── BIAS SAMPLING ──
int biasSampleCount = 0;
float biasSumGyro[3] = { 0, 0, 0 };
float biasSumAcc[3] = { 0, 0, 0 };

// ── GLOBALLY STORE GYRO DATA ──
float readingsGyro[3] = { 0, 0, 0 };

// ── GLOBALLY STORE ACCELERATION DATA ──
float readingsAcc[3] = { 0, 0, 0 };

// ── VELOCITY ACCUMULATION (sub-pixel) ──
float mouseAccumX = 0, mouseAccumY = 0;

// ─── EEG Signal processing config ───
#define SAMPLE_RATE 512
#define INPUT_PIN1 A0 // EEG input only

// EEG Envelope Configuration for blink detection
#define ENVELOPE_WINDOW_MS 100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Double/Triple Blink Configuration
const unsigned long BLINK_DEBOUNCE_MS = 250;
const unsigned long DOUBLE_BLINK_MS = 600;
unsigned long lastBlinkTime = 0;
unsigned long firstBlinkTime = 0;
unsigned long secondBlinkTime = 0;
unsigned long triple_blink_ms = 800;
int blinkCount = 0;
bool blinkActive = false;

float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;
float BlinkThreshold = 50.0;

// ─── FILTERS ───

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz
// Filter is order 2, implemented as second-order sections (biquads)
class NotchFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;
  BiquadState state1;

public:
  float process(float input)
  {
    float output = input;

    // Biquad section 0
    float x0 = output - (-1.58696045f * state0.z1) - (0.96505858f * state0.z2);
    output = 0.96588529f * x0 + -1.57986211f * state0.z1 + 0.96588529f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    // Biquad section 1
    float x1 = output - (-1.62761184f * state1.z1) - (0.96671306f * state1.z2);
    output = 1.00000000f * x1 + -1.63566226f * state1.z1 + 1.00000000f * state1.z2;
    state1.z2 = state1.z1;
    state1.z1 = x1;

    return output;
  }

  void reset()
  {
    state0.z1 = state0.z2 = 0;
    state1.z1 = state1.z2 = 0;
  }
} eegNotchFilter; // Only one filter needed for EEG

// High-Pass Butterworth IIR digital filter
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz
class EOGFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;

public:
  float process(float input)
  {
    float output = input;

    // Biquad section 0
    float x0 = output - (-1.91327599f * state0.z1) - (0.91688335f * state0.z2);
    output = 0.95753983f * x0 + -1.91507967f * state0.z1 + 0.95753983f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset()
  {
    state0.z1 = state0.z2 = 0;
  }
} eogFilter;

// Low-Pass Butterworth IIR digital filter
// Sampling rate: 512.0 Hz, frequency: 45.0 Hz
class EEGFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;

public:
  float process(float input)
  {
    float output = input;

    // Biquad section 0
    float x0 = output - (-1.24200128f * state0.z1) - (0.45885207f * state0.z2);
    output = 0.05421270f * x0 + 0.10842539f * state0.z1 + 0.05421270f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset()
  {
    state0.z1 = state0.z2 = 0;
  }
} eegFilter;

float updateEnvelope(float sample)
{
  float absSample = fabsf(sample);
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return envelopeSum / ENVELOPE_WINDOW_SIZE;
}

// ─── VIBRATION FEEDBACK FUNCTIONS ───
void startVibration()
{
  digitalWrite(VIBRATION_PIN, HIGH);
}

void stopVibration()
{
  digitalWrite(VIBRATION_PIN, LOW);
}

// Pick dominant axis from an accumulated gesture vector
void resolveAxis(float sum[3], int &axisIndex, int &axisSign) {
  int a = 0;
  if (fabs(sum[1]) > fabs(sum[a])) a = 1;
  if (fabs(sum[2]) > fabs(sum[a])) a = 2;
  axisIndex = a;
  axisSign = (sum[a] > 0) ? 1 : -1;
}

// ── BLE LED state machine ──
enum LedState
{
  LED_RED,
  LED_GREEN,
  LED_BLUE_FADE
};
LedState ledState = LED_RED;
unsigned long lastCmdSentMs = 0;
uint32_t lastPixel0Color = 0xFFFFFFFF;
static bool pixelDirty = false;

// ─── NON-BLOCKING CALIBRATION STATE MACHINE ───
void updateCalibrationStateMachine(unsigned long nowMs)
{
  if (calState == CAL_IDLE || calState == CAL_COMPLETE)
    return;

  unsigned long elapsed = nowMs - calStateStartTime;

  switch (calState)
  {
  case CAL_INIT_WAIT:
    if (elapsed >= 3000) {
      gestureSum[0] = gestureSum[1] = gestureSum[2] = 0;
      lastGyroMicros = micros();
      calState = CAL_UP_VIBRATE;
      calStateStartTime = nowMs;
      startVibration();
    }
    break;

  case CAL_UP_VIBRATE:
    {
      unsigned long n = micros();
      float dt = (n - lastGyroMicros) / 1000000.0f;
      lastGyroMicros = n;
      gestureSum[0] += readingsGyro[0] * dt;
      gestureSum[1] += readingsGyro[1] * dt;
      gestureSum[2] += readingsGyro[2] * dt;
      if (elapsed >= 3000) {
        stopVibration();
        resolveAxis(gestureSum, yAxisIndex, yAxisSign);
        yAxisSign = -yAxisSign;
        calState = CAL_UP_WAIT;
        calStateStartTime = nowMs;
      }
      break;
    }

  case CAL_UP_WAIT:
    if (elapsed >= 3000) {
      gestureSum[0] = gestureSum[1] = gestureSum[2] = 0;
      lastGyroMicros = micros();
      calState = CAL_LEFT_VIBRATE;
      calStateStartTime = nowMs;
      startVibration();
    }
    break;

  case CAL_LEFT_VIBRATE:
    {
      unsigned long n = micros();
      float dt = (n - lastGyroMicros) / 1000000.0f;
      lastGyroMicros = n;
      gestureSum[0] += readingsGyro[0] * dt;
      gestureSum[1] += readingsGyro[1] * dt;
      gestureSum[2] += readingsGyro[2] * dt;
      if (elapsed >= 3000) {
        stopVibration();
        resolveAxis(gestureSum, xAxisIndex, xAxisSign);
        xAxisSign = -xAxisSign;
        if (xAxisIndex == yAxisIndex)
          Serial.println("Both axes coincide - Calibration FAILED");
        calState = CAL_LEFT_WAIT;
        calStateStartTime = nowMs;
        axisCalibrated = true;
      }
      break;
    }

  case CAL_LEFT_WAIT:
    if (elapsed >= 2000) {
      biasSampleCount = 0;
      biasSumGyro[0] = biasSumGyro[1] = biasSumGyro[2] = 0;
      biasSumAcc[0] = biasSumAcc[1] = biasSumAcc[2] = 0;
      calState = CAL_NEUTRAL_SAMPLE;
      calStateStartTime = nowMs;
    }
    break;

  case CAL_NEUTRAL_SAMPLE:
    if (biasSampleCount < GYRO_BIAS_SAMPLES) {
      biasSumGyro[0] += readingsGyro[0];
      biasSumGyro[1] += readingsGyro[1];
      biasSumGyro[2] += readingsGyro[2];
      biasSumAcc[0] += readingsAcc[0];
      biasSumAcc[1] += readingsAcc[1];
      biasSumAcc[2] += readingsAcc[2];
      biasSampleCount++;
    } else {
      gyroBias[0] = biasSumGyro[0] / biasSampleCount;
      gyroBias[1] = biasSumGyro[1] / biasSampleCount;
      gyroBias[2] = biasSumGyro[2] / biasSampleCount;

      float accBias[3];
      accBias[0] = biasSumAcc[0] / biasSampleCount;
      accBias[1] = biasSumAcc[1] / biasSampleCount;
      accBias[2] = biasSumAcc[2] / biasSampleCount;

      {
        int gravAxis = 3 - xAxisIndex - yAxisIndex;
        float fwd = accBias[yAxisIndex] * yAxisSign;
        float side = accBias[xAxisIndex] * xAxisSign;
        float grav = accBias[gravAxis];
        neutralPitch = atan2(-fwd, sqrt(side * side + grav * grav)) * 180.0 / PI;
        neutralRoll = atan2(side, sqrt(fwd * fwd + grav * grav)) * 180.0 / PI;
      }

      isMPUCalibrated = true;
      calState = CAL_COMPLETE;

      for (int i = 0; i < 3; i++) {
        startVibration();
        delay(100);
        stopVibration();
        delay(100);
      }
    }
    break;

  default:
    break;
  }
}

void getAccelerometerAngles(float &pitch, float &roll) {
  int gravAxis = 3 - xAxisIndex - yAxisIndex;
  float fwd = readingsAcc[xAxisIndex] * xAxisSign;
  float side = readingsAcc[yAxisIndex] * yAxisSign;
  float grav = readingsAcc[gravAxis];
  pitch = atan2(-fwd, sqrt(side * side + grav * grav)) * 180.0 / PI;
  roll = atan2(side, sqrt(fwd * fwd + grav * grav)) * 180.0 / PI;
}

float mapAngleToMouse(float Angle) {
  float absAngle = fabs(Angle);
  float sign = (Angle > 0) ? 1.0f : -1.0f;
  float norm = constrain(absAngle / (MAX_TILT_ANGLE - DEADZONE), 0.0f, 1.0f);
  if (absAngle <= DEADZONE) return 0;
  float accel = pow(norm, ACCEL_CURVE);
  float speed = MIN_SENSITIVITY + (MAX_SENSITIVITY - MIN_SENSITIVITY) * accel * ACCEL_MULTIPLIER;
  return sign * speed;
}

// ─── PRECISION MOUSE CONTROL ───
void updatePrecisionMouse(unsigned long nowMs)
{
  if (!isMPUCalibrated || !axisCalibrated)
    return;

  if (nowMs - lastMouseUpdate < MOUSE_UPDATE_RATE)
    return;
  lastMouseUpdate = nowMs;

  float currentPitch, currentRoll;
  getAccelerometerAngles(currentPitch, currentRoll);

  float deltaPitch = currentPitch - neutralPitch;
  float deltaRoll = currentRoll - neutralRoll;

  if (fabs(deltaPitch) < DEADZONE) deltaPitch = 0;
  if (fabs(deltaRoll) < DEADZONE) deltaRoll = 0;

  float targetVelX = mapAngleToMouse(deltaRoll);
  float targetVelY = mapAngleToMouse(deltaPitch);

  mouseVelocityX = MOVEMENT_SMOOTHING * mouseVelocityX + (1.0f - MOVEMENT_SMOOTHING) * targetVelX;
  mouseVelocityY = MOVEMENT_SMOOTHING * mouseVelocityY + (1.0f - MOVEMENT_SMOOTHING) * targetVelY;

  if (fabs(targetVelX) < 0.01f) {
    mouseVelocityX *= VELOCITY_DECAY;
    if (fabs(mouseVelocityX) < STOP_THRESHOLD) mouseVelocityX = 0;
  }
  if (fabs(targetVelY) < 0.01f) {
    mouseVelocityY *= VELOCITY_DECAY;
    if (fabs(mouseVelocityY) < STOP_THRESHOLD) mouseVelocityY = 0;
  }

  mouseAccumX += mouseVelocityX;
  mouseAccumY += mouseVelocityY;
  int finalMouseX = (int)mouseAccumX;
  int finalMouseY = (int)mouseAccumY;
  mouseAccumX -= finalMouseX;
  mouseAccumY -= finalMouseY;

  if (finalMouseX != 0 || finalMouseY != 0)
  {
    Mouse.move(finalMouseX, finalMouseY);
    lastCmdSentMs = millis();
    ledState = LED_BLUE_FADE;
  }
}

// ── Battery level ──
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;
static unsigned long lastBatteryCheck = -10000;
uint32_t batteryColor = 0;
static uint32_t batteryWinSum = 0;
static uint16_t batteryWinCount = 0;
static int lastBatteryPct = -1;
static uint8_t risingCount = 0;
static const uint8_t RISING_THRESHOLD = 3;
const float voltageLUT[] = {
    3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
    3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20};
const int percentLUT[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
    50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

float interpolatePercentage(float voltage)
{
  if (voltage <= voltageLUT[0])
    return 0;
  if (voltage >= voltageLUT[lutSize - 1])
    return 100;
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1])
    i++;
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

int getCurrentBatteryPercentage()
{
  float avgRaw = (batteryWinCount > 0) ? (batteryWinSum / batteryWinCount) : analogRead(BATTERY_VOLTAGE_PIN);
  batteryWinSum = 0;
  batteryWinCount = 0;
  float voltage = (avgRaw / 1000.0) * 2;
  voltage += 0.022;
  float percentage = interpolatePercentage(voltage);
  if (lastBatteryPct == -1)
  {
    lastBatteryPct = (int)percentage;
  }
  else if ((int)percentage < lastBatteryPct)
  {
    lastBatteryPct = (int)percentage;
    risingCount = 0;
  }
  else if ((int)percentage > lastBatteryPct)
  {
    risingCount++;
    if (risingCount >= RISING_THRESHOLD)
    {
      lastBatteryPct = (int)percentage;
      risingCount = 0;
    }
  }
  else
  {
    risingCount = 0;
  }
  return lastBatteryPct;
}

// ========== BLINK DETECTION (double blink = left click, triple blink = right click) ==========
void handleBlinks(unsigned long nowMs)
{
  bool envelopeHigh = currentEEGEnvelope > BlinkThreshold;
  if (!blinkActive && envelopeHigh && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS)
  {
    lastBlinkTime = nowMs;
    if (blinkCount == 0)
    {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS)
    {
      secondBlinkTime = nowMs;
      blinkCount = 2;
    }
    else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= triple_blink_ms)
    {
      Mouse.click(MOUSE_RIGHT);
      if (Keyboard.isConnected())
      {
        lastCmdSentMs = millis();
        ledState = LED_BLUE_FADE;
      }
      blinkCount = 0;
    }
    else
    {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    blinkActive = true;
  }

  if (!envelopeHigh)
  {
    blinkActive = false;
  }

  // Double blink timeout -> Left mouse click
  if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms)
  {
    Mouse.click(MOUSE_LEFT);
    if (Keyboard.isConnected())
    {
      lastCmdSentMs = millis();
      ledState = LED_BLUE_FADE;
    }
    blinkCount = 0;
  }
  // Single blink timeout
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS)
  {
    blinkCount = 0;
  }
}

// Update IMU I2C led
void updateIMULed(bool connectionStatus)
{
  uint32_t color;

  if (!connectionStatus)
  {
    color = pixel.Color(20, 0, 0); // Red = IMU communication failed
  }
  else
  {
    color = pixel.Color(0, 20, 0); // Green = ready
  }
  pixel.setPixelColor(IMU_LED, color);
  pixel.show();
}

void updateBLELed()
{
  uint32_t color;
  if (ledState == LED_RED)
  {
    color = pixel.Color(20, 0, 0);
  }
  else if (ledState == LED_GREEN)
  {
    color = pixel.Color(0, 20, 0);
  }
  else
  {
    unsigned long elapsed = millis() - lastCmdSentMs;
    if (elapsed < BLUE_LED_DURATION)
    {
      color = pixel.Color(0, 0, 30);
    }
    else
    {
      ledState = LED_GREEN;
      color = pixel.Color(0, 20, 0);
    }
  }
  if (color != lastPixel0Color)
  {
    lastPixel0Color = color;
    pixel.setPixelColor(BLE_LED, color);
    pixel.show();
  }
}

// ─── setup() ───
void setup()
{
  pixel.begin();
  pixel.clear();
  pixel.show();

  Wire.begin(22, 23);
  // Initialize MPU6050 failed

  int currentBattery = getCurrentBatteryPercentage();
  if (currentBattery <= 20)
  {
    batteryColor = pixel.Color(20, 0, 0);
  }
  else if (currentBattery <= 70)
  {
    batteryColor = pixel.Color(30, 20, 0);
  }
  else
  {
    batteryColor = pixel.Color(0, 20, 0);
  }
  pixel.setPixelColor(BATTERY_LED, batteryColor);

  while (!mpu.begin())
  {
    Serial.print("MPU6050 initialization FAILED!");
    static uint16_t fader = 100;
    static bool decreasing = true;
    pixel.setPixelColor(IMU_LED, pixel.Color(fader, 0, 0));
    pixel.show();
    delay(20);
    if (decreasing)
    {
      fader = fader - 2;
      if (fader < 10)
      {
        decreasing = false;
      }
    }
    else
    {
      fader = fader + 2;
      if (fader > 100)
      {
        decreasing = true;
      }
    }
  }

  // MPU initialized
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // START NON-BLOCKING CALIBRATION
  calState = CAL_INIT_WAIT;
  calStateStartTime = millis();

  // Initialize ADC pins - only EEG channel
  pinMode(INPUT_PIN1, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);

  // Initialize vibration motor (OFF)
  digitalWrite(VIBRATION_PIN, LOW);

  // Initialize BLE Combo (Keyboard + Mouse) - no custom name parameter
  Keyboard.begin();
  Mouse.begin();
}

// ─── loop() ───
void loop()
{
  bool connected = Keyboard.isConnected();
  static bool lastConnected = false;
  if (connected != lastConnected)
  {
    lastConnected = connected;
    ledState = connected ? LED_GREEN : LED_RED;
    pixelDirty = true;
  }

  Wire.beginTransmission(mpuAddress);

  bool imuConnected;
  if (!Wire.endTransmission())
  {
    imuConnected = true;
  }
  else
  {
    imuConnected = false;
  }
  static bool lastConnection = false;
  if (imuConnected != lastConnection)
  {
    lastConnection = imuConnected;
    updateIMULed(imuConnected);
    pixelDirty = true;
    if (!imuConnected)
    {
      isMPUCalibrated = false;
      axisCalibrated = false;
      calState = CAL_IDLE;

      mouseVelocityX = 0;
      mouseVelocityY = 0;
      mouseAccumX = 0;
      mouseAccumY = 0;

      eegNotchFilter.reset();
      eogFilter.reset();
      eegFilter.reset();
      Serial.print("MPU disconnected - calibration invalidated");
    }
    else
    {
      if (mpu.begin())
      {
        Serial.print("MPU reconnected - restarting calibration");
        calState = CAL_INIT_WAIT;
        calStateStartTime = millis();
      }
      else
      {
        Serial.print("MPU reconnected but beginI2C() failed");
      }
    }
  }

  static unsigned long lastMicros = micros();

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;
  static long timer = 0;
  timer -= dt;

  if (timer <= 0)
  {
    timer += 1000000L / SAMPLE_RATE;
    unsigned long nowMs = millis();

    // Read sensor data into globals
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    readingsGyro[0] = g.gyro.x - gyroBias[0];
    readingsGyro[1] = g.gyro.y - gyroBias[1];
    readingsGyro[2] = g.gyro.z - gyroBias[2];
    readingsAcc[0] = a.acceleration.x;
    readingsAcc[1] = a.acceleration.y;
    readingsAcc[2] = a.acceleration.z;

    // NON-BLOCKING CALIBRATION UPDATE
    updateCalibrationStateMachine(nowMs);

    // 1) EEG ADC read - only one channel
    int raw1 = analogRead(INPUT_PIN1);
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    // 2) EEG filtering and envelope for blinks
    float filteeg = eegFilter.process(eegNotchFilter.process(raw1));
    float filteog = eogFilter.process(filteeg);
    currentEEGEnvelope = updateEnvelope(filteog);

    if (connected)
    {
      handleBlinks(nowMs);
    }
  }

  // 4) PRECISION MOUSE CONTROL - runs continuously
  if (connected)
  {
    updatePrecisionMouse(millis());
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL)
  {
    int currentBattery = getCurrentBatteryPercentage();
    if (currentBattery <= 20)
    {
      batteryColor = pixel.Color(20, 0, 0);
    }
    else if (currentBattery <= 70)
    {
      batteryColor = pixel.Color(30, 20, 0);
    }
    else
    {
      batteryColor = pixel.Color(0, 20, 0);
    }
    pixelDirty = true;
    lastBatteryCheck = currentMillis;
  }

  if (pixelDirty)
  {
    pixel.setPixelColor(BATTERY_LED, batteryColor);
    lastPixel0Color = 0xFFFFFFFF;
    pixelDirty = false;
  }
  updateBLELed();
}
