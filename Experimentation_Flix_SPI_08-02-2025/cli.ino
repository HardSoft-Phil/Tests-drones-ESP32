// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Implementation of command line interface

#include "pid.h"
#include "vector.h"

extern PID rollRatePID, pitchRatePID, yawRatePID, rollPID, pitchPID;
extern LowPassFilter<Vector> ratesFilter;
extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;

const char* motd =
  "======================================\n"
  "Commandes:\n"
  "--------------------------------------\n"
  "help -> aide\n"
  "p ----> tous les paramètres\n"
  "p <name> --> montre le parametre\n"
  "p <name> <value> --> ajuste les paramètres\n"
  "preset --> raz des paramètres\n"
  "ps ---> pitch/roll/yaw\n"
  "psq --> attitude quaternion\n"
  "imu --> données IMU\n"
  "rc  --> données RC\n"
  "mot --> données moteurs\n"
  "log --> dump in-RAM log\n"
  "cr ---> calibre RC\n"
  "cg ---> calibre gyro\n"
  "ca ---> calibre accel\n"
  "avd, avg, ard, arg --> test des moteurs\n"
  "reset --> raz état du drone\n"
  "reboot --> reboote le drone\n"
  "======================================\n";

//=============================================================
void doCommand(String& command, String& arg0, String& arg1) {
  if (command == "help" || command == "motd") {
    Serial.println(motd);
  } else if (command == "p" && arg0 == "") {
    printParameters();
  } else if (command == "p" && arg0 != "" && arg1 == "") {
    Serial.printf("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
  } else if (command == "p") {
    bool success = setParameter(arg0.c_str(), arg1.toFloat());
    if (success) {
      Serial.printf("%s = %g\n", arg0.c_str(), arg1.toFloat());
    } else {
      Serial.printf("Parameter not found: %s\n", arg0.c_str());
    }
  } else if (command == "preset") {
    resetParameters();
  } else if (command == "ps") {
    Vector a = attitude.toEulerZYX();
    Serial.printf("roll: %f pitch: %f yaw: %f\n", a.x * RAD_TO_DEG, a.y * RAD_TO_DEG, a.z * RAD_TO_DEG);
  } else if (command == "psq") {
    Serial.printf("qx: %f qy: %f qz: %f qw: %f\n", attitude.x, attitude.y, attitude.z, attitude.w);
  } else if (command == "imu") {
    printIMUInfo();
    Serial.printf("gyro: %f %f %f\n", rates.x, rates.y, rates.z);
    Serial.printf("acc: %f %f %f\n", acc.x, acc.y, acc.z);
    printIMUCal();
    Serial.printf("rate: %f\n", loopRate);
  } else if (command == "rc") {
    Serial.printf("Raw: throttle %d yaw %d pitch %d roll %d armed %d mode %d\n",
                  channels[RC_CHANNEL_THROTTLE], channels[RC_CHANNEL_YAW], channels[RC_CHANNEL_PITCH],
                  channels[RC_CHANNEL_ROLL], channels[RC_CHANNEL_ARMED], channels[RC_CHANNEL_MODE]);
    Serial.printf("Control: throttle %f yaw %f pitch %f roll %f armed %f mode %f\n",
                  controls[RC_CHANNEL_THROTTLE], controls[RC_CHANNEL_YAW], controls[RC_CHANNEL_PITCH],
                  controls[RC_CHANNEL_ROLL], controls[RC_CHANNEL_ARMED], controls[RC_CHANNEL_MODE]);
    Serial.printf("Mode: %s\n", getModeName());
  } else if (command == "mot") {
    Serial.printf("MOTOR front-right %f front-left %f rear-right %f rear-left %f\n",
                  motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
  } else if (command == "log") {
    dumpLog();
  } else if (command == "cr") {
    calibrateRC();
  } else if (command == "cg") {
    calibrateGyro();
  } else if (command == "ca") {
    calibrateAccel();
  } else if (command == "avd") {
    cliTestMotor(MOTOR_FRONT_RIGHT);
  } else if (command == "avg") {
    cliTestMotor(MOTOR_FRONT_LEFT);
  } else if (command == "ard") {
    cliTestMotor(MOTOR_REAR_RIGHT);
  } else if (command == "arg") {
    cliTestMotor(MOTOR_REAR_LEFT);
  } else if (command == "reset") {
    attitude = Quaternion();
  } else if (command == "reboot") {
    ESP.restart();
  } else if (command == "") {
    // do nothing
  } else {
    Serial.println("Invalid command: " + command);
  }
}

//=============================================================
void cliTestMotor(uint8_t n) {
  Serial.printf("Testing motor %d\n", n);
  motors[n] = 1;
  delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
  sendMotors();
  delay(3000);
  motors[n] = 0;
  sendMotors();
  Serial.println("Done");
}

//=============================================================
void parseInput() {
  static bool showMotd = true;
  static String input;

  if (showMotd) {
    Serial.println("\n======================================");
    Serial.println(version);
    Serial.println(motd);
    showMotd = false;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      char chars[input.length() + 1];
      input.toCharArray(chars, input.length() + 1);
      String command = stringToken(chars, " ");
      String arg0 = stringToken(NULL, " ");
      String arg1 = stringToken(NULL, "");
      doCommand(command, arg0, arg1);
      input.clear();
    } else {
      input += c;
    }
  }
}

//=============================================================
// Helper function for parsing input
String stringToken(char* str, const char* delim) {
  char* token = strtok(str, delim);
  return token == NULL ? "" : token;
}

//=============================================================
