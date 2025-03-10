// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Revu et corrigé JPC 06/02/2025
// A l'origine: axe des Y vers l'avant, capteur à l'envers.
// Modifié pour: axe des X vers l'avant, capteur à l'endroit

// Work with the IMU sensor

#include <Wire.h>
#include <src/MPU9250.h>

MPU9250 IMU(SPI);

Vector accBias;
Vector gyroBias;
Vector accScale(1, 1, 1);

extern const float ONE_G;

//=============================================================
void setupIMU() {
  Serial.println("Setup IMU");
  bool status = IMU.begin();
  if (!status) {
    while (true) {
      Serial.println("IMU begin error");
      delay(1000);
    }
  }
  configureIMU();
  delay(500); // wait a bit before calibrating
  calibrateGyro();
}

//=============================================================
void configureIMU() {
  IMU.setAccelRange(IMU.ACCEL_RANGE_4G);
  IMU.setGyroRange(IMU.GYRO_RANGE_2000DPS);
  IMU.setDLPF(IMU.DLPF_MAX);
  IMU.setRate(IMU.RATE_1KHZ_APPROX);
}

//=============================================================
void readIMU() {
  IMU.waitForData();
  IMU.getGyro(gyro.x, gyro.y, gyro.z);
  IMU.getAccel(acc.x, acc.y, acc.z);
  // apply scale and bias
  acc = (acc - accBias) / accScale;
  gyro = gyro - gyroBias;
  // rotate
  rotateIMU(acc);
  rotateIMU(gyro);
}

//=============================================================
void rotateIMU(Vector& data) {
  // Rotate from LFD to FLU
  // NOTE: In case of using other IMU orientation, change this line:
  //  data = Vector(data.y, data.x, -data.z);
  data = Vector(data.x, data.y, data.z);
  // Axes orientation for various boards: https://github.com/okalachev/flixperiph#imu-axes-orientation
}

//=============================================================
void calibrateGyro() {
  const int samples = 1000;
  Serial.println("Calibrating gyro, stand still");
  IMU.setGyroRange(IMU.GYRO_RANGE_250DPS);  // the most sensitive mode

  gyroBias = Vector(0, 0, 0);
  for (int i = 0; i < samples; i++) {
    IMU.waitForData();
    IMU.getGyro(gyro.x, gyro.y, gyro.z);
    gyroBias = gyroBias + gyro;
  }
  gyroBias = gyroBias / samples;

  printIMUCal();
  configureIMU();
}

//=============================================================
void calibrateAccel() {
  Serial.println("Calibrating accelerometer");
  IMU.setAccelRange(IMU.ACCEL_RANGE_2G);  // the most sensitive mode

  Serial.setTimeout(60000);
  Serial.print("Place level [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place nose up [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place nose down [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place on right side [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place on left side [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();
  Serial.print("Place upside down [enter] ");
  Serial.readStringUntil('\n');
  calibrateAccelOnce();

  printIMUCal();
  configureIMU();
}

//=============================================================
void calibrateAccelOnce() {
  const int samples = 1000;
  static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
  static Vector accMin(INFINITY, INFINITY, INFINITY);

  // Compute the average of the accelerometer readings
  acc = Vector(0, 0, 0);
  for (int i = 0; i < samples; i++) {
    IMU.waitForData();
    Vector sample;
    IMU.getAccel(sample.x, sample.y, sample.z);
    acc = acc + sample;
  }
  acc = acc / samples;

  // Update the maximum and minimum values
  if (acc.x > accMax.x) accMax.x = acc.x;
  if (acc.y > accMax.y) accMax.y = acc.y;
  if (acc.z > accMax.z) accMax.z = acc.z;
  if (acc.x < accMin.x) accMin.x = acc.x;
  if (acc.y < accMin.y) accMin.y = acc.y;
  if (acc.z < accMin.z) accMin.z = acc.z;
  Serial.printf("acc %f %f %f\n", acc.x, acc.y, acc.z);
  Serial.printf("max %f %f %f\n", accMax.x, accMax.y, accMax.z);
  Serial.printf("min %f %f %f\n", accMin.x, accMin.y, accMin.z);
  // Compute scale and bias
  accScale = (accMax - accMin) / 2 / ONE_G;
  accBias = (accMax + accMin) / 2;
}

//=============================================================
void printIMUCal() {
  Serial.printf("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
  Serial.printf("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
  Serial.printf("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

//=============================================================
void printIMUInfo() {
  Serial.printf("model: %s\n", IMU.getModel());
  Serial.printf("who am I: 0x%02X\n", IMU.whoAmI());
}

//=============================================================
