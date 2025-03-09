#include "src/SBUS/sbus.h"

// Objet SBUS, initialise la voie de réception
bfs::SbusRx sbus_rx(&Serial1, SBUS_RX, SBUS_TX, true);
/* SBUS data */
bfs::SbusData data;

//####################################################
void RCSetup() {
  // Initialise la communication SBUS
  sbus_rx.Begin();
}

//####################################################
void loop_sbus () {
  if (sbus_rx.Read()) {
    // Enregistre le message que s'il est reçu
    data = sbus_rx.data();
    // Ordre standard message sbus télécommande Microzone
    rcValue[ROL]  = map(data.ch[0], 200, 1800, 1000, 2000); // Roll
    rcValue[PIT]  = map(data.ch[1], 200, 1800, 1000, 2000); // Pitch
    rcValue[THR]  = map(data.ch[2], 200, 1800, 1000, 2000); // Trotlle
    rcValue[RUD]  = map(data.ch[3], 200, 1800, 1000, 2000); // Yaw
    rcValue[AUX1] = map(data.ch[4], 200, 1800, 1000, 2000); // Aux1
    rcValue[AUX2] = map(data.ch[5], 200, 1800, 1000, 2000); // Aux2
  }
}

//####################################################
// Vérifie s'il y a une connection avec l'émetteur
bool checkRemote()
{ // On ne mesure que la valeur de throtlle si pas d'émission 
  if (rcValue[THR] > 950) return true;
  else return false;
}

//#####################################################
