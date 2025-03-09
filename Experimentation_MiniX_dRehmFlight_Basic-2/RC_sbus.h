#include "src/SBUS/sbus.h"

// Objet SBUS, initialise la voie de réception
bfs::SbusRx sbus_rx(&Serial1, SBUS_RX, SBUS_TX, true);
/* SBUS data */
bfs::SbusData data;

// Remote signal timeout: 1 seconde
#define SIGNAL_TIMEOUT 1000
static unsigned long lastRecvTime = 0;
bool radio_ON = false;

//####################################################
void RCSetup() {
  // Initialise la communication SBUS
  sbus_rx.Begin();
}

//####################################################
void loop_sbus () {
  //  data = {0,0,0,0,0,0};
  if (sbus_rx.Read()) {
    // Enregistre le message s'il est reçu
    data = sbus_rx.data();
    // Ordre standard message sbus télécommande Microzone
    channel_2_pwm = map(data.ch[0], 200, 1800, 1000, 2000);// Roll
    channel_3_pwm = map(data.ch[1], 200, 1800, 1000, 2000);// Pitch
    channel_1_pwm = map(data.ch[2], 200, 1800, 1000, 2000);// Trotlle
    channel_4_pwm = map(data.ch[3], 200, 1800, 1000, 2000);// Yaw
    channel_5_pwm = map(data.ch[4], 200, 1800, 1000, 2000);// Aux1
    channel_6_pwm = map(data.ch[5], 200, 1800, 1000, 2000);// Aux2
    lastRecvTime  = millis();
  }
}

//####################################################
// Vérifie s'il y a une connection avec l'émetteur
void checkRemote() {
  //  unsigned long Now = millis();
  //  if ( Now - lastRecvTime > SIGNAL_TIMEOUT ) return true;
  if (rcValue[THR] > 950) return true;
  else radio_ON = false;
}

//#####################################################
