/*
  float mapf(long x, long in_min, long in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
  }

  float mapff(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  int8_t sign(float x) {
  return (x > 0) - (x < 0);
  }
*/
//##########################################################
// Gestion led rouge, clignotement
//  - paramètre d'entrée: uint32_t frequence
//  - pas de valeur retournée
//##########################################################
void ledRBlink(uint32_t frequence)
{
  if ((millis() - RledTimer) >= frequence)
  {
    RledTimer = millis();
    RblinkState = !RblinkState;
    digitalWrite(LEDR_PIN, RblinkState);
  }
}

//##########################################################
// Gestion led verte, clignotement
//  - paramètre d'entrée: uint32_t frequence
//  - pas de valeur retournée
//##########################################################
void ledVBlink(uint32_t frequence)
{
  if ((millis() - VledTimer) >= frequence)
  {
    VledTimer = millis();
    VblinkState = !VblinkState;
    digitalWrite(LEDV_PIN, VblinkState);
  }
}

//##########################################################
// Gestion led bleue, clignotement
//  - paramètre d'entrée: uint32_t frequence
//  - pas de valeur retournée
//##########################################################
void ledBlink(uint32_t frequence)
{
  if ((millis() - ledTimer) >= frequence)
  {
    ledTimer = millis();
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
/*
//##########################################################
// Mesure tension batterie
//  - pas de paramètre d'entrée
//  - valeur retournée float
// ESP32 pas assez stable pour mesurer une tension analogique
// https://electroniqueamateur.blogspot.com/2019/08/esp32-utilisation-des-entrees.html
// https://forum.arduino.cc/t/lecture-tension-sur-esp32-incorrecte/923360/52?page=2
//##########################################################
float getBat() // A vérifier
{ // R1 = 47K, R2 = 10K, Vbat max = 12V
  // Vout mesuré = (Vbat*R2)/(R1+R2) = 0,893617021 pour 1V
  // Donc, (4.4 / 4096) * 0,893617021
  float vBat = 0;
  for (int i = 1; i <= 5; i++)  {
    vBat += analogRead(V_BAT_PIN);
    delay(10);
  }
  vBat = vBat / 5; // Moyenne sur 5 mesures
  return (7, 68E-4 * vBat + 0, 1734); // ?? A vérifier
}
*/
//##########################################################
//  Process RadioCommande
//  Changement de l'état du drone
//  - pas de paramètre d'entrée
//  - aucune valeur retournée
//##########################################################
void processRC()
{
  if (rcValue[AUX1] == true) modeVol = ACCRO;
  else modeVol = STABLE;
  if (oldmode != modeVol) // Si le mode a changé
  {
    zeroGyroAccI();
    oldmode = modeVol;
  }

  //  if (rcValue[AUX2] == true) camMode = HIGH;
  //  else camMode = LOW;
  //  digitalWrite(CAM_PIN, camMode);

  // Quand le stick de gauche est positionné dans le coin inférieur droit
  if (etat == ARRET && rcValue[THR] < 1050 && rcValue[RUD] < 1050)
  {
    etat = ARME;
  }

  // Quand le stick de gauche revient au centre
  if (etat == ARME && rcValue[RUD] >= 1450 && rcValue[THR] < 1050)
  {
    etat = MARCHE;
    // Ré-initialise les variables du PID pour éviter un démarrage chaotique
    zeroGyroAccI();
  }

  // Quand le stick de gauche est positionné dans le coin inférieur gauche
  if (etat == MARCHE && rcValue[RUD] > 1950 && rcValue[THR] < 1050)
  {
    etat = ARRET;
    onMoteurs(); // On s'assure de stopper les moteurs
  }

  if (etat == ARME || etat == MARCHE)
  {
    ledVBlink(100); // Led verte clignote très rapidement
    ledRBlink(100); // Led rouge clignote très rapidement
    rcCommand[ROLL]  = rcValue[ROL] - MIDCOMMAND;
    rcCommand[PITCH] = rcValue[PIT] - MIDCOMMAND;
    rcCommand[YAW]   = rcValue[RUD] - MIDCOMMAND;
  } else
  {
    ledVBlink(750); // Led verte clignote lentement
    ledRBlink(750); // Led rouge clignote lentement
  }
}

//##########################################################
void loopRate(int freq) { // Issu de dRhemFlight
  // DESCRIPTION : Régule la fréquence de la boucle principale à une fréquence
  // spécifiée en Hz.

  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

//##########################################################
void failSafe() {
  // DESCRIPTION: Si la liaison radio donne des valeurs erronées,
  // effectue la procédure d'atterissage d'urgence
  if (checkRemote() == false) // Si plus de signal radio
  { // procédure d'atterissage d'urgence (trop rapide)
    modeVol = STABLE;        // mode STABLE
    if (millis() - t1Hz > 1000000)
    {
    --rcValue[THR];       // Décrémente la vitesse
      t1Hz = millis();
      if(rcValue[THR] >= MINCOMMAND) MINCOMMAND;
    }
  }
}

//##############################################################################
// Moniteur série pour visualisation et modification des paramètres de vol.
//##############################################################################
// Saisie des paramètres sur le port série.
int readsernum()
{
  int num;
  char numStr[3];
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}

//##########################################################
void moniteurRS (void)
{
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 'E')
    {
      Serial.println("Début d'étalonnage MPU");
      calculate_IMU_error();
//      ACC_Store();
      Serial.println("Étalonnage MPU Terminé");
    }
    else if (ch == 'M') {
      Serial.println("Début d'étalonnage Magnéto");
      calibrateMagnetometer();
//      MPU_Store();
      Serial.println("Étalonnage Magnéto Terminé");
    }
        else if (ch == 'A')
        {
          Serial.print("Paramètres actuels :\n");
          Serial.print("PID Accro :  ");
          Serial.print(P_PID); Serial.print("\t");
          Serial.print(I_PID); Serial.print("\t");
          Serial.print(D_PID); Serial.println();
          Serial.print("PID Stable : ");
          Serial.print(P_Level_PID); Serial.print("\t");
          Serial.print(I_Level_PID); Serial.print("\t");
          Serial.print(D_Level_PID); Serial.println();
        }
        else if (ch == 'D')
        {
          Serial.println("Charge le PID par Défaut");
          yawRate = 6.0;
          rollPitchRate = 5.0;
          P_PID = 0.15;    // P8
          I_PID = 0.00;    // I8
          D_PID = 0.08;
          P_Level_PID = 0.35;   // P8
          I_Level_PID = 0.00;   // I8
          D_Level_PID = 0.10;
//          PID_Store();
        }
        else if (ch == 'W')
        {
          char ch = Serial.read();
          int n = Serial.available();
          if (n == 3)
          {
            n = readsernum();
            if      (ch == 'p') {
              P_PID       = float(n) * 0.01 + 0.004;
              Serial.print("pid P ");
              Serial.print(P_PID);
            }
            else if (ch == 'i') {
              I_PID       = float(n) * 0.01 + 0.004;
              Serial.print("pid I ");
              Serial.print(I_PID);
            }
            else if (ch == 'd') {
              D_PID       = float(n) * 0.01 + 0.004;
              Serial.print("pid D ");
              Serial.print(D_PID);
            }
            else if (ch == 'P') {
              P_Level_PID = float(n) * 0.01 + 0.004;
              Serial.print("pid Level P ");
              Serial.print(P_Level_PID);
            }
            else if (ch == 'I') {
              I_Level_PID = float(n) * 0.01 + 0.004;
              Serial.print("pid Level I ");
              Serial.print(I_Level_PID);
            }
            else if (ch == 'D') {
              D_Level_PID = float(n) * 0.01 + 0.004;
              Serial.print("pid Level D ");
              Serial.print(D_Level_PID);
            }
            else Serial.println("Commande inconnue");
          }
          else if (ch == 'S') {
//            PID_Store();
//            Serial.print("PID sauvé en EEPROM");
          }
          else
          {
            Serial.println("Mauvais format d'entrée");
            Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
            Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
          }
        }
        else if (ch >= '0' && ch <= '9') debugvalue = ch - '0';
    if (ch == 'h') // Affichage de l'aide
    {
      Serial.print("======================================================\n");
      Serial.print("------------------------MiniX-------------------------\n");
      Serial.print("  Version : "); Serial.println(version);
      Serial.print("======================================================\n");
      Serial.println("h - HELP");
      Serial.println("E -> Etalonnage Axel/Gyro");
      Serial.println("M -> Etalonnage Mag");
      Serial.println("D -> Ecrit le PID par Défaut");
      Serial.println("A -> Valeurs PID Actuel");
      Serial.println("Wpxx, Wixx, Wdxx -> Ecrit PID accro");
      Serial.println("WPxx, WIxx, WDxx, WPxx ou WRxx -> Ecrit PID stable");
      Serial.println("WS - Sauve PID en EEPROM");
      Serial.print("------------------------------------------------------\n");
      Serial.println("            Affiche les données:");
      Serial.println("0 -> Arret défilement");
      Serial.println("1 -> Valeurs Gyro");
      Serial.println("2 -> Valeurs Acc");
      Serial.println("3 -> Valeurs Angle");
      Serial.println("4 -> Valeurs Mag");
      Serial.println("5 -> Valeurs RC");
      Serial.println("6 -> Valeurs Moteurs");
      Serial.print("======================================================\n");
    }
  }

  if      (debugvalue == 1) Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);
  else if (debugvalue == 2) Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
  else if (debugvalue == 3) Serial.printf("%3f %3f \n", angle[PITCH], angle[ROLL]);
  else if (debugvalue == 5)
  {
    if (checkRemote() == true) Serial.print("RC ON\t");
    else Serial.print("RC OFF\t");
    if (etat == 0) Serial.print(F("Arret\t"));
    if (etat == 1) Serial.print(F("Armé\t"));
    if (etat == 2) Serial.print(F("Marche\t"));
    if (modeVol == STABLE) Serial.print("Mode stable\t");
    else Serial.print("Mode accro\t");
//    Serial.println(camMode ? "Cam On" : "Cam Off");
 Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]);  }
  else if (debugvalue == 6)
  {
    Serial.printf("%4.2f, %4.2f, %4.2f, %4.2f\n", servo[AVD], servo[ARD], servo[ARG], servo[AVG]);
  }
//  else if (debugvalue == 7)
//  {
//    float vBat = getBat();
//    Serial.printf("%1.2f Volts\n", vBat);
//    debugvalue = 0;
//  }
//  else if (debugvalue == 8)
//  {
//  }
}

//##########################################################################
