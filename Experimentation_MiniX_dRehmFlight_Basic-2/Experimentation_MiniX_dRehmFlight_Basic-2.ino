// #################################################################
// #################################################################
// ####                 MiniX Flight Controller                 ####
// #################################################################
// #### J-P Chazarin inspiré par:                               ####
// ####     https://github.com/nickrehm/dRehmFlight             ####
// ####     et avec des idées de:                               ####
// ####     https://github.com/okalachev/flix                   ####
// ####     https://github.com/PepeTheFroggie/EspCopter32       ####
// #################################################################
// #### Compatible au choix avec :                              ####
// ####   * ESP32-S2-Drone V1.2 (ESP32-s2, MPU6050 I2c)         ####
// ####   * WEMOS_D1_mini_ESP32 (ESP WROOM 32, MPU 9250)        ####
// ####  Télécommande au choix sbus ou esp_now                  ####
// ####  Quatre moteurs au choix courant continu ou brushless   ####
// #################################################################
// #################################################################
// Essai de fonctionnement sur la base de:
//  Expérimentation_MiniX_dRehmFlight du 22/02/2025
//  et dRehmFlight version 1.2. qui est un fork de MadFlight
//
// #################################################################
// #################################################################
// 25/02/2025:
//  -> Tout à l'air de fonctionner, a tester plus en détail !
//=============================================================
// 25/02/2025:
//  -> Ajout stockage paramètres en eeprom => a mettre au point !
//  -> Attention voie d'entrée sbus changée !!!
//=============================================================

#include "Params.h"

#define DEBUG

//==================================================================
char *version = "Expérimentation_MiniX_dRehmFlight_Basic1 du 25/02/2025";
//==================================================================
//####################################################
//####                  SETUP()                   ####
//####################################################
void setup() {
  //Initialise les leds
  pinMode(LEDR_PIN, OUTPUT);
  pinMode(LEDV_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  //#ifdef ONESHOT
  pinMode(MOTPIN_1, OUTPUT);
  pinMode(MOTPIN_2, OUTPUT);
  pinMode(MOTPIN_3, OUTPUT);
  pinMode(MOTPIN_4, OUTPUT);
  //#endif

  digitalWrite(LEDR_PIN, HIGH);
  digitalWrite(LEDV_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  delay(1000);

#ifdef DEBUG
  Serial.begin(115200);
  delay(500);
  Serial.printf("\nVersion : %s\n\n", version);
  Serial.print("*** Initialisation ***\n");
#endif
  disableBrownOut();

  initMoteurs(); // Ici pour éviter que les moteurs "chantent".

  // Initialise radio-communication
  RCSetup();

  // Initialise IMU
  IMUinit();

  delay(500);

  // Mesure l'erreur IMU sur les lectures d'accéléromètre et de
  // gyroscope zéro, en supposant que le véhicule est à niveau
  //calculate_IMU_error();

  // Si IMU MPU9250, étalonnage unique du magnétomètre
  // (il faudra peut-être répéter l'opération pour les nouveaux emplacements)
  //calibrateMagnetometer();

  EEPROM.begin(117);
  //  if (EEPROM.read(116) != 0xAB) Serial.println("Etalonnage MPU indispensable");
  //  else MPU_Read();      // Lecture du PID en eeprom
  //  if (EEPROM.read(115) != 0xCD) Serial.println("Besoin de vérifier ou d'écrire le PID\n");
  //  else PID_Read();      // Lecture des paramètres en eeprom

  // aide à réchauffer l'IMU et le filtre Madgwick avant le loop
  calibrateAttitude();
  zeroPID();

#ifdef DEBUG
  Serial.println("*** Initialisation terminée ***");
#endif
}

//####################################################
//####                    LOOP()                  ####
//####################################################
void loop() {
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;

  // Extrait les données brutes du gyroscope, de l'accéléromètre et du magnétomètre
  // vers les filtres IMU et LP pour supprimer le bruit.
  getIMUdata();
  // Met à jour roll_IMU, pitch_IMU et yaw_IMU (degrés)
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);

#ifdef MINIX_WROOM
  loop_sbus();
#endif

  // Calcule l'état désiré
  getDesState(); // Convertit les commandes radio brutes en valeurs normalisées
  //  getCommands(); // Filtre les commandes radio => Pas bon
  // Controlleurs PID - EN SELECTIONNER UN:
  controlANGLE();    // Mode stable (stabilise on angle setpoint)
  //  controlRATE(); // Mode accro (stabilise on rate setpoint)

  // Mixe et met à l'échelle les moteurs en fonction des valeurs PWM
  controlMixer();  // Mixage des données PID
  scaleMotors(); // Mise à l'échelle des commandes moteur
  throttleCut(); //directly sets motor commands to low if non armed

  // Envoie les impulsions de commande aux moteurs
  //#ifdef ONESHOT
  //  runOneShot();
  //#else
  runMoteurs();
  //#endif

  checkRemote(); // Vérifie s'il y a une connection avec la télécommande

  failSafe(); // Procédure d'atterissage d'urgence en cas de mauvaise connexion au récepteur

#ifdef DEBUG
  moniteurSerie();
#endif

  // Regulation temporelle du loop
  // Les paramètres des filtres sont pré-réglés sur 2000Hz
  loopRate(2000);
}

//####################################################
