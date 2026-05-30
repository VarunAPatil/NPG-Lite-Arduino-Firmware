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
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN           15
#define BATTERY_VOLTAGE_PIN A6

Adafruit_NeoPixel pixel(6, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BATTERY_LED 5

Adafruit_MPU6050 mpu;

// ———— CONFIG ————
// Invert if an axis feels backwards:
const float INV_X =  1.0;  // +1 = normal, –1 = flip X
const float INV_Y =  1.0;  // +1 = normal, –1 = flip Y
const float INV_Z =  1.0;  // +1 = normal, –1 = flip Z

// How many samples to take for your “zero” calibration:
const int   CAL_SAMPLES = 200;
const float CAL_DELAY_MS = 5.0;  // wait between reads

// These will be filled in by calibrateOffsets():
float offsetX = 0.0;
float offsetY = 0.0;
float offsetZ = 0.0;

void calibrateOffsets() {
  float sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t a, g, temp;

  Serial.println("Calibrating… keep the board perfectly level and still");

  for (int i = 0; i < CAL_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(CAL_DELAY_MS);
  }
  offsetX = sumX / CAL_SAMPLES;
  offsetY = sumY / CAL_SAMPLES;
  offsetZ = sumZ / CAL_SAMPLES;

  Serial.print("Offsets → X: ");
  Serial.print(offsetX, 3);
  Serial.print("  Y: ");
  Serial.println(offsetY, 3);
  Serial.print("  Z: ");
  Serial.println(offsetZ, 3);
  Serial.println("Calibration done!");
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
  pixel.begin();
  pixel.clear();
  pixel.show();

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(230400);
  while (!Serial) {}  // wait for USB‑Serial

  if (!mpu.begin()) {
    Serial.println("MPU not found!");
    while (1) { delay(10); }
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  // give sensor time to settle
  delay(100);

  calibrateOffsets();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply your calibration & optional inversion:
  float ax = (a.acceleration.x - offsetX) * INV_X;
  float ay = (a.acceleration.y - offsetY) * INV_Y;
  float az = (a.acceleration.z - offsetZ) * INV_Z;

  // Print only X and Y (comma‑separated):
  char buf[64];
  int len = snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f\n", ax, ay, az);
  Serial.write(buf, len);

  // Throttle rate if you like:
  // delay(5);

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