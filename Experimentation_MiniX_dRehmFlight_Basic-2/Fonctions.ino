#include <soc/rtc_cntl_reg.h>

extern bool radio_ON;
//=============================================================
// Désactive le reboot en cas de baisse de tension
void disableBrownOut() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
}

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
// Mesure tension batterie
//  - pas de paramètre d'entrée
//  - valeur retournée float
//##########################################################
float getBat()
{ // R1 = 47K, R2 = 10K, Vbat max = 4.4V
  // Vout mesuré = (Vbat*R2)/(R1+R2) = 0,893617021 pour 1V
  // Donc, (4.2 / 4096) * 0,893617021
  float vBat = 0;
  for (int i = 1; i <= 5; i++)  vBat += analogRead(V_BAT_PIN);
  vBat = vBat / 5; // Moyenne sur 5 mesures
  //  https://electroniqueamateur.blogspot.com/2019/08/esp32-utilisation-des-entrees.html
  return (7, 68E-4 * vBat + 0, 1734); // ?? A vérifier
}

//##########################################################
void getDesState() {
  //DESCRIPTION: Normalise les valeurs de contrôle souhaitées aux valeurs appropriées
  //    Met à jour les variables d'état souhaitées thro_des, roll_des, pitch_des et
  //    yaw_des. Celles-ci sont calculées en utilisant les commandes RC pwm brutes
  //    et en les mettant à l'échelle pour être dans les limites définies dans la
  //    configuration. thro_des reste dans la plage de 0 à 1. roll_des et pitch_des
  //    sont mis à l'échelle pour être dans la quantité maximale de roulis/tangage
  //    en degrés (mode angle) ou en degrés/sec (mode taux). Yaw_des est mis à l'échelle
  //    pour être dans le lacet maximal en degrés/sec.

  if (!armed && channel_4_pwm < 1050 && channel_1_pwm < 1050) {
    started = true;
  }
  if (started && channel_4_pwm >= 1450 && channel_1_pwm < 1050) {
    // Ré-initialise les variables du PID pour éviter un démarrage chaotique
    zeroPID();
    onMoteurs();
    armed = true;
  }
  if (armed && channel_4_pwm >= 1550 && channel_1_pwm < 1050) {
    onMoteurs(); // On s'assure de stopper tous les moteurs
    armed = false;
    started = false;
  }

  thro_des = (channel_1_pwm - 1000.0) / 1000.0; //between 0 and 1
  roll_des = (channel_2_pwm - 1500.0) / 500.0; //between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0) / 500.0; //between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0) / 500.0; //between -1 and 1
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll; //between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; //between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw; //between -maxYaw and +maxYaw

  //  roll_passthru = roll_des / (2 * maxRoll);
  //  pitch_passthru = pitch_des / (2 * maxPitch);
  //  yaw_passthru = yaw_des / (2 * maxYaw);

  if (armed) { // Signalisation lumineuse
    ledVBlink(100); // Led verte clignote très rapidement
    ledRBlink(100); // Led rouge clignote très rapidement
  } else {
    ledVBlink(750); // Led verte clignote lentement
    ledRBlink(750); // Led rouge clignote lentement
  }
}

//##########################################################
void getCommands() {
  // DESCRIPTION: Get raw PWM values for every channel from the radio
  // Low-pass the critical commands and update previous values
  float b = 0.2; //lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

//####################################################
void controlMixer() {
  // Quad mixing
  // m1 = front right, m2 = back right, m3 = back left, m1 = front left
  m1_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;
  m2_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;
  m3_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;
  m4_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;
}

//##########################################################
void scaleMotors() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  //     mX_command_scaled variables are scaled to 125-250us for OneShot125 protocol.
  //      to 0-180 for the brushed motors and to 1000-2000 for standard ESC using standard PWM.
  if (armed) {
#if defined ONESHOT
    //Scaled to 125us - 250us for oneshot125 protocol
    m1_command_PWM = m1_command_scaled * 125 + 125;
    m2_command_PWM = m2_command_scaled * 125 + 125;
    m3_command_PWM = m3_command_scaled * 125 + 125;
    m4_command_PWM = m4_command_scaled * 125 + 125;
#elif defined BRUSHED
    // Scaled to 0 - 250 for brushed motors
    m1_command_PWM = m1_command_scaled * 250;
    m2_command_PWM = m2_command_scaled * 250;
    m3_command_PWM = m3_command_scaled * 250;
    m4_command_PWM = m4_command_scaled * 250;
#else if defined PWM
    // Scaled to 1000 - 2000 for PWM protocol (ESC)
    m1_command_PWM = m1_command_scaled * 1000 + 1000;
    m2_command_PWM = m2_command_scaled * 1000 + 1000;
    m3_command_PWM = m3_command_scaled * 1000 + 1000;
    m4_command_PWM = m4_command_scaled * 1000 + 1000;
#endif
    //Constrain commands to specifics motors
    servo[AVD] = constrain(m1_command_PWM, MOT_MIN, MOT_MAX);
    servo[ARD] = constrain(m2_command_PWM, MOT_MIN, MOT_MAX);
    servo[ARG] = constrain(m3_command_PWM, MOT_MIN, MOT_MAX);
    servo[AVG] = constrain(m4_command_PWM, MOT_MIN, MOT_MAX);
  } else onMoteurs();
}

//##########################################################
void failSafe() {
  // DESCRIPTION: Si la liaison radio donne des valeurs erronées,
  // effectue la procédure d'atterissage d'urgence
  if (!radio_ON) { // Si plus de signal radio reçu
    if (millis() - t1Hz > 1000000)
    {
      //    controlANGLE();  // Mode stable
      channel_1_pwm-- ; // Décrémente la vitesse
      channel_2_pwm = channel_2_fs;
      channel_3_pwm = channel_3_fs;
      channel_4_pwm = channel_4_fs;
      channel_5_pwm = channel_5_fs;
      t1Hz = millis();
    }
  }
}

//##########################################################
void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  if (!armed) {
    m1_command_PWM = MOT_MIN;
    m2_command_PWM = MOT_MIN;
    m3_command_PWM = MOT_MIN;
    m4_command_PWM = MOT_MIN;
  }
}

//##########################################################
void loopRate(int freq) {
  // DESCRIPTION : Régule la fréquence de la boucle principale à une fréquence
  // spécifiée en Hz.
  //  Il est bon de fonctionner à une fréquence de boucle constante pour que les
  //  filtres restent stables et autres. Les routines d'interruption exécutées en
  //  arrière-plan font fluctuer la fréquence de la boucle. Cette fonction attend
  //  simplement à la fin de chaque itération de boucle jusqu'à ce que le temps
  //  correct se soit écoulé depuis le début de la boucle actuelle pour la
  //  fréquence de boucle souhaitée en Hz. 2 kHz est une bonne fréquence à atteindre
  //  car la boucle fonctionnera nominalement entre 2, 8 kHz et 4, 2 kHz. Cela nous
  //  permet d'avoir un peu de marge pour ajouter des calculs supplémentaires et
  //  rester au-dessus de 2 kHz, sans avoir besoin de réajuster tous nos paramètres
  //  de filtrage dispersés dans ce morceau de code.

  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

//=========================================================================================//
//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
  */
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

//##########################################################
float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq) {
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*
      Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency
      and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
      and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being
      monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical
      statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
      to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.

  */
  float diffParam = (param_max - param_min) / (fadeTime * loopFreq); //difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //constrain param within max bounds

  return param;
}

//##########################################################
float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq) {
  //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*
      Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency
      and linearly interpolates that param variable up or down to the desired value. This function can be called in controlMixer()
      to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being
      monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical
      statements in order to fade controller gains, for example between the two dynamic configurations.

  */
  //float diffParam_up = (param_upper - param_des)/(fadeTime_up*loopFreq); //difference to add to param for each loop iteration for desired fadeTime_up
  //float diffParam_down = (param_des - param_lower)/(fadeTime_down*loopFreq); //difference to subtract from param for each loop iteration for desired fadeTime_down

  if (param > param_des) { //need to fade down to get to desired
    float diffParam = (param_upper - param_des) / (fadeTime_down * loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { //need to fade up to get to desired
    float diffParam = (param_des - param_lower) / (fadeTime_up * loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper); //constrain param within max bounds

  return param;
}

//##########################################################
float switchRollYaw(int reverseRoll, int reverseYaw) {
  //DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*
     Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
     Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not
     reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis.
     This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the
     IMU tilted 90 degrees from default level).
  */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}

//##########################################################
