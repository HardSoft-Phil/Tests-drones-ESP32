extern bool radio_ON;

//###########################################################################
//# Moniteur série pour visualisation et modification des paramètres de vol #
//###########################################################################
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
void moniteurSerie(void)
{
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 'E') {
      Serial.println("Début d'étalonnage Axel/Gyro");
      calculate_IMU_error();
      MPU_Store();
      Serial.println("Étalonnage Axel/Gyro Terminé");
    }
    else if (ch == 'M') {
      Serial.println("Début d'étalonnage Magnéto");
      calibrateMagnetometer();
      MPU_Store();
      Serial.println("Étalonnage Magnéto Terminé");
    }
    else if (ch == 'A') {
      Serial.print("Paramètres PID actuels :\n");
      Serial.print("PID Accro :\tP: ");
      Serial.print(Kp_roll_rate); Serial.print("\tI: ");
      Serial.print(Ki_roll_rate); Serial.print("\tD: ");
      Serial.print(Kd_roll_rate); Serial.print("\n");
      Serial.print("PID Stable :\tP: ");
      Serial.print(Kp_pitch_angle); Serial.print("\tI: ");
      Serial.print(Ki_pitch_angle); Serial.print("\tD: ");
      Serial.print(Kd_pitch_angle); Serial.print("\tB_roll: ");
      Serial.print(B_loop_roll); Serial.print("\tB_pitch: ");
      Serial.print(B_loop_pitch); Serial.print("\n");
      Serial.print("PID Yaw :\tP: ");
      Serial.print(Kp_yaw); Serial.print("\tI: ");
      Serial.print(Ki_yaw); Serial.print("\tD: ");
      Serial.print(Kd_yaw); Serial.print("\n");
      debugvalue = 0;
    }
    else if (ch == 'D') {
      Serial.println("Charge le PID par Défaut");
      Kp_roll_angle = 0.2;
      Ki_roll_angle = 0.3;
      Kd_roll_angle = 0.05;
      B_loop_roll = 0.9;
      Kp_pitch_angle = 0.2;
      Ki_pitch_angle = 0.3;
      Kd_pitch_angle = 0.05;
      B_loop_pitch = 0.9;
      //      PID_Store();
    }
    else if (ch == 'W') {
      char ch = Serial.read();
      int n = Serial.available();
      if (n == 3) {
        n = readsernum();
        if (ch == 'p') {
          Kp_roll_angle = float(n) * 0.01 + 0.004;
          Serial.print("P PID accro: ");
          Serial.print(Kp_roll_angle);
        }
        else if (ch == 'i') {
          Ki_roll_angle = float(n) * 0.01 + 0.004;
          Serial.print("I PID accro: ");
          Serial.print(Ki_roll_angle);
        }
        else if (ch == 'd') {
          Kd_roll_angle = float(n) * 0.01 + 0.004;
          Serial.print("D PID accro: ");
          Serial.print(Kd_roll_angle);
        }
        else if (ch == 'P') {
          Kp_pitch_angle = float(n) * 0.01 + 0.004;
          Serial.print("P PID stable: ");
          Serial.print(Kp_pitch_angle);
        }
        else if (ch == 'I') {
          Ki_pitch_angle = float(n) * 0.01 + 0.004;
          Serial.print("I PID stable: ");
          Serial.print(Ki_pitch_angle);
        }
        else if (ch == 'D') {
          Kd_pitch_angle = float(n) * 0.01 + 0.004;
          Serial.print("D PID stable: ");
          Serial.print(Kd_pitch_angle);
        }
        else if (ch == 'P') {
          B_loop_pitch = float(n) * 0.01 + 0.004;
          Serial.print("B_loop pitch stable: ");
          Serial.print(B_loop_pitch);
        }
        else if (ch == 'R') {
          B_loop_roll = float(n) * 0.01 + 0.004;
          Serial.print("B_loop roll stable: ");
          Serial.print(B_loop_roll);
        }
        else Serial.print("Commande inconnue\n");
      }
      else if (ch == 'S') {
        PID_Store();
        Serial.print("PID sauvé en mémoire");
      }
      else {
        Serial.println("Mauvais format d'entrée");
        Serial.println("Wpxx, Wixx, Wdxx - Ecrit PID accro, exemple: Wd13");
        Serial.println("WPxx, WIxx, WDxx, WPxx ou WRxx - Ecrit PID stable, exemple: WD21");
      }
    }
    else if (ch >= '0' && ch <= '9') debugvalue = ch - '0';
    if (ch == 'h') { // Affichage de l'aide
      Serial.print("======================================================\n");
      Serial.println(version);
      Serial.print("======================================================\n");
      Serial.println("h -> HELP");
      Serial.println("E -> Etalonnage Axel/Gyro");
      Serial.println("M -> Etalonnage Mag");
      Serial.println("D -> Ecrit le PID par Défaut");
      Serial.println("A -> Valeurs PID Actuel");
      Serial.println("Wpxx, Wixx, Wdxx -> Ecrit PID accro");
      Serial.println("WPxx, WIxx, WDxx, WPxx ou WRxx -> Ecrit PID stable");
      //      Serial.println("WS -> Sauve PID en EEPROM");
      Serial.print("------------------------------------------------------\n");
      Serial.println("            Affiche les données:");
      Serial.println("0 -> Arret défilement");
      Serial.println("1 -> Valeurs Gyro");
      Serial.println("2 -> Valeurs Acc");
      Serial.println("3 -> Roll, Pitch, Yaw");
      Serial.println("4 -> Valeurs Mag");
      Serial.println("5 -> Valeurs RC");
      Serial.println("6 -> Valeurs Moteurs");
      Serial.println("7 -> Voltage Bat");
      //Serial.println("8 -> Altitude");
      Serial.println("9 -> fréquence");
      Serial.print("======================================================\n");
    }
  }
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    if      (debugvalue == 1) Serial.printf("%4f %4f %4f \n", GyroX, GyroY, GyroZ);
    else if (debugvalue == 2) Serial.printf("%4f %4f %4f \n", AccX, AccY, AccZ);
    else if (debugvalue == 3) Serial.printf("%3f %3f %3f\n", roll_IMU, pitch_IMU, yaw_IMU);
    else if (debugvalue == 4) Serial.printf("%4f %4f %4f\n", MagX, MagY, MagZ);
    else if (debugvalue == 5) {
      Serial.print(radio_ON ? "RC ON\t" : "RC OFF\t");
      Serial.print(armed ? "Marche\t" : "Arret\t");
      Serial.printf("%4d %4d %4d %4d\n", channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm);
    }
    else if (debugvalue == 6) {
      Serial.printf("%4d, %4d, %4d, %4d\n", servo[0], servo[2], servo[3], servo[1]);
    }
    else if (debugvalue == 7) {
      float vBat = getBat();
      Serial.printf("%3.2f Volts\n", vBat);
      debugvalue = 0;
    }
    else if (debugvalue == 8) {
    }
    else if (debugvalue == 9) { // print Rate
    Serial.print(F("dt = "));
    Serial.println(dt*1000000.0);
    }
  }
}

//##########################################################################
