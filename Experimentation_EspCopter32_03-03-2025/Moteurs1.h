// Gestion des moteurs en PWM

#define PWM_FREQUENCY 250
#define PWM_RESOLUTION 8

#define MOT_MAX 1900
#define MOT_MIN 1000

//=============================================================
void onMoteurs()
{
  servo[AVD] = MOT_MIN;
  servo[ARD] = MOT_MIN;
  servo[ARG] = MOT_MIN;
  servo[AVG] = MOT_MIN;
}

//=============================================================
void stopMoteurs()
{
  servo[AVD] = 0.0;
  servo[ARD] = 0.0;
  servo[ARG] = 0.0;
  servo[AVG] = 0.0;
}

//=============================================================
void runMoteurs()
{
  ledcWrite(MOTPIN_1, servo[AVD]); // Moteur Avant Droit
  ledcWrite(MOTPIN_2, servo[ARD]); // Moteur Arrière Droit
  ledcWrite(MOTPIN_3, servo[ARG]); // Moteur Arriere Gauche
  ledcWrite(MOTPIN_4, servo[AVG]); // Moteur Avant Gauche
}

//=============================================================
void mix()
{
  if (etat == MARCHE & (rcValue[2] > 1050))
  { // Moteurs brushed
    servo[AVD] = round(map(rcValue[2] - axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW], 1000, 2000, MINCOMMAND, MOT_MAX));
    servo[ARD] = round(map(rcValue[2] + axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW], 1000, 2000, MOT_MIN, MOT_MAX));
    servo[ARG] = round(map(rcValue[2] + axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW], 1000, 2000, MOT_MIN, MOT_MAX));
    servo[AVG] = round(map(rcValue[3] - axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW], 1000, 2000, MOT_MIN, MOT_MAX));
  }
  else onMoteurs();

  // Envoie les impulsions de commande aux moteurs
  runMoteurs();
}

//=============================================================
void initMoteurs()
{ // Mise en route des moteurs pour éviter un plantage au démarrage
  ledcAttach(MOTPIN_1, PWM_FREQUENCY, PWM_RESOLUTION);// AV-D
  ledcAttach(MOTPIN_2, PWM_FREQUENCY, PWM_RESOLUTION);// AR-D
  ledcAttach(MOTPIN_3, PWM_FREQUENCY, PWM_RESOLUTION);// AR-G
  ledcAttach(MOTPIN_4, PWM_FREQUENCY, PWM_RESOLUTION);// AV-G
  onMoteurs();   // Transmission des commandes aux moteurs
  runMoteurs();

  axisPID[0] = 0; axisPID[1] = 0; axisPID[2] = 0;
  delay(100);
}

//##########################################################
