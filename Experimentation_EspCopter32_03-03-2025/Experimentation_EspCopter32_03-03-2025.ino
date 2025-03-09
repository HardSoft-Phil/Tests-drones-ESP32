// ################################################################
// ################################################################
// ####           Programme d'expérimentation MiniX            ####
// ####  Version WEMOS_D1_mini moteurs pilotés en mode direct  ####
// ################################################################
// #### J-P Chazarin inspiré par:                              ####
// ####     https://github.com/PepeTheFroggie/EspCopter32      ####
// ####     et avec des idées de:                              ####
// ####     https://github.com/nickrehm/dRehmFlight            ####
// ####     https://github.com/okalachev/flix                  ####
// ################################################################
// #### * Carte WEMOS_D1_mini_ESP32:                           ####
// ####   -> processeur ESP32 WROOM 32                         ####
// ####   -> module MPU 9250                                   ####
// ####   -> Télécommande SBUS                                 ####
// ####   -> 4 x moteurs (brushed ou brushless)                ####
// ################################################################
// 19/01/2025:
// -> Reprise de Quadri_18-01-2025 pour expérimenter sans casser ce qui marche!
// -> Réorganisation du mix moteurs.
// -> Tout parait correct. Reste à fignoler les coef du PID...
// ----------------------------------------------------------------
// 04/03/2025:
// -> Changement IMU, MPU9250 commandé en SPI
// -> Télécommande SBUS
// -> Gestion moteurs brushless protocole OneShot125
// ----------------------------------------------------------------
//==================================================================

#include "Params.h"

//=============================================================
char *version = "Expérimentation_EspCopter-SPI du 04/03/2025";
//=============================================================

//#######################################################
//####                     SETUP()                   ####
//#######################################################
void setup()
{
  pinMode(LEDR_PIN, OUTPUT);
  pinMode(LEDV_PIN, OUTPUT);
  pinMode(LED_PIN,  OUTPUT);

  digitalWrite(LEDR_PIN, HIGH);
  digitalWrite(LEDV_PIN, HIGH);
  ledBlink(200);
//  digitalWrite(CAM_PIN , LOW);
  delay(500);

#ifdef DEBUG
  Serial.begin(115200); Serial.println();
  while (!Serial) {
    delay(10);
  }
  Serial.printf("\nVersion : %s\n\n", version);
  Serial.print("*** Initialisation ***\n");
#endif
  initMoteurs();          // Setup moteurs
  RCSetup();              // Setup Radio-Commande
  delay(1000);            // Pause de stabilisation
  IMUinit();              // Initialisation du MPU
  delay(100);             // Petite pause
  calculate_IMU_error();  // Calcul erreurs MPU
  digitalWrite(LEDR_PIN, LOW);
  etat = ARRET;           // Par sécurité
  zeroGyroAccI();         // RaZ du PID
  Serial.println("*** Initialisation terminée ***");
}

//####################################################
//####                    LOOP()                  ####
//####################################################
void loop()
{
  loop_sbus();            // Récupération message radio
  processRC();            // Interpretation commandes radio
  getIMUdata();           // Récupère les données MPU
  EstimeAttitude();       // Calcule l'attitude du drone
  pid();                  // Calculs PID de reactions du drone
  failSafe();             // Procédure d'atterissage d'urgence en cas de coupure radio
  mix();                  // Mélange les données PID et les envoie aux moteurs
#ifdef DEBUG              // Moniteur série si DEBUG activé
  moniteurRS();
#endif
  // Regulation temporelle du loop
  // Les paramètres des filtres sont pré-réglés sur 2000Hz
  loopRate(2000);
}

//####################################################
