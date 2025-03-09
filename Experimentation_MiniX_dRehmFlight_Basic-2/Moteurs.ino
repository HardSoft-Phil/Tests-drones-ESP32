// Gestion de moteurs brushed
// Motors output control using MOSFETs

// Pins moteurs dans "src/MiniX_pins_D1_mini_ESP32.h"

#define PWM_FREQUENCY 250
#define PWM_RESOLUTION 8
//##########################################################
void onMoteurs()
{
  servo[AVD] = MOT_MIN;
  servo[ARD] = MOT_MIN;
  servo[ARG] = MOT_MIN;
  servo[AVG] = MOT_MIN;
}

//##########################################################
void runMoteurs()
{
  ledcWrite(MOTPIN_1, servo[AVD]); // Moteur Avant Droit
  ledcWrite(MOTPIN_3, servo[ARD]); // Moteur Arrière Droit
  ledcWrite(MOTPIN_4, servo[ARG]); // Moteur Arriere Gauche
  ledcWrite(MOTPIN_2, servo[AVG]); // Moteur Avant Gauche
}

//##########################################################
void initMoteurs()
{
  ledcAttach(MOTPIN_1, PWM_FREQUENCY, PWM_RESOLUTION);// AV-D
  ledcAttach(MOTPIN_2, PWM_FREQUENCY, PWM_RESOLUTION);// AR-D
  ledcAttach(MOTPIN_3, PWM_FREQUENCY, PWM_RESOLUTION);// AR-G
  ledcAttach(MOTPIN_4, PWM_FREQUENCY, PWM_RESOLUTION);// AV-G

  // Mise en route des moteurs pour éviter un plantage au démarrage
  onMoteurs();   // Transmission des commandes aux moteurs
  delay(500);
}

//##########################################################
void runOneShot() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;

  //Write all motor pins high
  digitalWrite(MOTPIN_1, HIGH);
  digitalWrite(MOTPIN_2, HIGH);
  digitalWrite(MOTPIN_3, HIGH);
  digitalWrite(MOTPIN_4, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 6 ) { //keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ((servo[AVD] <= timer - pulseStart) && (flagM1 == 0)) {
      digitalWrite(MOTPIN_1, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((servo[ARD] <= timer - pulseStart) && (flagM2 == 0)) {
      digitalWrite(MOTPIN_2, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((servo[ARG] <= timer - pulseStart) && (flagM3 == 0)) {
      digitalWrite(MOTPIN_3, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((servo[AVG] <= timer - pulseStart) && (flagM4 == 0)) {
      digitalWrite(MOTPIN_4, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    }
  }
}

//##########################################################
