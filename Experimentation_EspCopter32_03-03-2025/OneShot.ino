
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
float m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
extern unsigned long rcValue[]; extern int16_t axisPID[];
//##########################################################
void onMoteurs()
{
  m1_command_scaled = 125; //command OneShot125 ESC from 125 to 250us pulse length
  m2_command_scaled = 125;
  m3_command_scaled = 125;
  m4_command_scaled = 125;
}

//##########################################################
void mix() {
 if (etat == MARCHE & (rcValue[THR] > 1050))
  { // Moteurs brushed
    m1_command_scaled = rcValue[THR] - axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW];
    m2_command_scaled = rcValue[THR] + axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW];
    m3_command_scaled = rcValue[THR] + axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW];
    m4_command_scaled = rcValue[THR] - axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW];
  }
  else onMoteurs();

  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  servo[AVD] = constrain(m1_command_PWM, 125, 250);
  servo[ARD] = constrain(m2_command_PWM, 125, 250);
  servo[ARG] = constrain(m3_command_PWM, 125, 250);
  servo[AVD] = constrain(m4_command_PWM, 125, 250);
  
  runOneShot(); 
}

//##########################################################
void initMoteurs()
{ // Mise en route des moteurs pour éviter un plantage au démarrage
  pinMode(MOTPIN_1, OUTPUT);
  pinMode(MOTPIN_2, OUTPUT);
  pinMode(MOTPIN_3, OUTPUT);
  pinMode(MOTPIN_4, OUTPUT);
  
  onMoteurs(); //Arm OneShot125 motors
  
  // Some oneshot ESCs do not arm with only one command pulse...
  runOneShot();
  delay(10);
  runOneShot();
  delay(10);
  runOneShot(); 
}

//##########################################################
void runOneShot() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  int flagM6 = 0;
  
  //Write all motor pins high
  digitalWrite(MOTPIN_1, HIGH);
  digitalWrite(MOTPIN_2, HIGH);
  digitalWrite(MOTPIN_3, HIGH);
  digitalWrite(MOTPIN_4, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 4 ) { //keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ((servo[AVD] <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(MOTPIN_1, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((servo[ARD] <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(MOTPIN_2, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((servo[ARG] <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(MOTPIN_3, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((servo[AVG] <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(MOTPIN_4, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
  }
}

//##########################################################
