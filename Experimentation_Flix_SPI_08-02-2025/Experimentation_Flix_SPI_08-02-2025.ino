//################################################################
//####           Programme d'expérimentation MiniX            ####
//####   Version ESP32_mini moteurs pilotés en mode direct    ####
//################################################################
//####             Algoritme Flix d'Oleg Kalachev             ####
//####            https://github.com/okalachev/flix           ####
//################################################################

//=============================================================
// 08/02/2025:
//  -> Francisation de l'aide
//  -> Ordre des moteurs changé: M0=AVD, M1=ARD, M2=ARG, M3=AVG
//=============================================================
//  A FAIRE:
//  -> Intégrer les paramètres spécifiques de la carte processeur
//      dans un fichier à part dans le répertoire "src"
//=============================================================

#include "vector.h"
#include "quaternion.h"

#define SERIAL_BAUDRATE 115200

// Choix de la télécommande
//#define WIFI_ENABLED
#define RC_ENABLED
//#define NOW_ENABLED

#define RC_CHANNELS 16
// Ordre des canaux de telecommande
#define RC_CHANNEL_ROLL     0
#define RC_CHANNEL_PITCH    1
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_YAW      3
#define RC_CHANNEL_ARMED    5
#define RC_CHANNEL_MODE     4

float t = NAN; // current step time, s
float dt; // time delta from previous step, s
float loopRate; // loop rate, Hz
int16_t channels[RC_CHANNELS]; // raw rc channels
float controls[RC_CHANNELS]; // normalized controls in range [-1..1] ([0..1] for throttle)
float controlsTime; // time of the last controls update
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
float motors[4]; // normalized motors thrust in range [-1..1]

//=============================================================
char *version = "Experimentation Flix_SPI du 08/02/2025";
//=============================================================
//                          SETUP
//=============================================================
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("Initializing flix");
  disableBrownOut();
  setupParameters();
  setupLED();
  setupMotors();
  setLED(true);
#ifdef WIFI_ENABLED
  setupWiFi();
#endif

#ifdef RC_ENABLED
  setupRC();
#elif defined NOW_ENABLED
  // Ordre des canaux de telecommande
  //  setupNow
#endif

  setupIMU();

  setLED(false);
  Serial.println("Initializing complete");
}

//=============================================================
//                          LOOP
//=============================================================
void loop() {
  readIMU();
  step();
#ifdef RC_ENABLED
  readRC();
#endif
  estimate();
  control();
  sendMotors();
  parseInput();
#ifdef WIFI_ENABLED
  processMavlink();
#endif
  logData();
  syncParameters();
}

//=============================================================
