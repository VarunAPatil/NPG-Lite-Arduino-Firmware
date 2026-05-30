// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Aman Maheshwari - aman@upsidedownlabs.tech 
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE       512           // samples per second
#define BAUD_RATE         115200
#define INPUT_PIN           A0
#define PIXEL_PIN           15
#define BATTERY_VOLTAGE_PIN A6

Adafruit_NeoPixel pixel(6, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BATTERY_LED 5

// EEG Envelope Configuration
#define ENVELOPE_WINDOW_MS 100  // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Double Blink Detection Configuration
const unsigned long BLINK_DEBOUNCE_MS   = 250;   // minimal spacing between individual blinks
const unsigned long DOUBLE_BLINK_MS     = 600;   // max time between the two blinks
unsigned long lastBlinkTime     = 0;             // time of most recent blink
unsigned long firstBlinkTime    = 0;             // time of the first blink in a pair
unsigned long secondBlinkTime   = 0;
unsigned long triple_blink_ms   = 600; 
int         blinkCount         = 0;             // how many valid blinks so far (0–2)

// EEG Processing Variables
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;
float BlinkThreshold = 75.0;

// --- Filter Functions ---
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float highpass(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.91327599*z1 - 0.91688335*z2;
    output = 0.95753983*x + -1.91507967*z1 + 0.95753983*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float Notch(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.58696045*z1 - 0.96505858*z2;
    output = 0.96588529*x + -1.57986211*z1 + 0.96588529*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.62761184*z1 - 0.96671306*z2;
    output = 1.00000000*x + -1.63566226*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// EEG Envelope Detection Function
float updateEEGEnvelope(float sample) {
  float absSample = fabs(sample);  // Rectify EEG signal

  // Update circular buffer and running sum
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return envelopeSum / ENVELOPE_WINDOW_SIZE;  // Return moving average
}

// ── Battery indicator (pixel 5) ──
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;
static unsigned long lastBatteryCheck = -10000;
const float voltageLUT[] = {
  3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
  3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
};
const int percentLUT[] = {
  0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
  50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};
const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

float interpolatePercentage(float voltage) {
  if (voltage <= voltageLUT[0]) return 0;
  if (voltage >= voltageLUT[lutSize - 1]) return 100;
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1]) i++;
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

int getCurrentBatteryPercentage() {
  int analogValue = analogRead(BATTERY_VOLTAGE_PIN);
  float voltage = (analogValue / 1000.0) * 2;
  voltage += 0.022;
  return (int)interpolatePercentage(voltage);
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(100);

  pixel.begin();
  pixel.clear();
  pixel.show();

  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;
  if(timer <= 0){
    timer += 1000000L / SAMPLE_RATE;
    int raw = analogRead(INPUT_PIN);
    float filt = highpass(Notch(raw));
    currentEEGEnvelope = updateEEGEnvelope(filt);
  }

  // Double blink detection
  unsigned long nowMs = millis();

  // 1) Did we cross threshold and respect per‑blink debounce?
  if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;    // mark this blink

    // 2) Count it
    if (blinkCount == 0) {
      // first blink of the pair
      firstBlinkTime = nowMs;
      blinkCount = 1;
      // Serial.println("First blink detected");
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      // double blink detected - send right arrow key
      secondBlinkTime = nowMs;
      blinkCount = 2;
      // Serial.println("Second blink registered, waiting for triple…");
    }
    else if (blinkCount==2 && (nowMs - secondBlinkTime) <= triple_blink_ms)
    {
      Serial.println("Triple blink detected!");
      blinkCount=0;
    }
    else {
      // either too late or extra blink → restart sequence
      firstBlinkTime = nowMs;
      blinkCount = 1;
      // Serial.println("Blink sequence restarted");
    }
  }

    // if we were in “2 blinks” but no third arrived in time → treat as a real double
    if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms) {
      Serial.println("Double blink detected");
      blinkCount = 0;
    }

  // 3) Timeout: if we never got the second blink in time, reset
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
    blinkCount = 0;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    int pct = getCurrentBatteryPercentage();
    uint32_t color;
    if (pct <= 20)      color = pixel.Color(20, 0, 0);
    else if (pct <= 70) color = pixel.Color(30, 20, 0);
    else                color = pixel.Color(0, 20, 0);
    pixel.setPixelColor(BATTERY_LED,color);
    pixel.show();
    lastBatteryCheck = currentMillis;
  }
}
