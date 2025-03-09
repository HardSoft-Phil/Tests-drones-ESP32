// ################################################################
// #### * Carte ESP32-Drone V1.2  comprenant:											   ####
// ####   -> processeur ESP32-S2 Wrover 														   ####
// ####   -> module MPU6050 																		   ####
// ####   -> 4 x moteurs courant continu (brushed) 										   ####
// #### * Télécommande par ESP_NOW  														   ####
// #### * pas de bluetooth sur ce µcontroleur *   											   ####
// ################################################################

//--------------I2c-----------------
#define I2C0_SDA 11
#define I2C0_SCL 10 

//#define I2C1_SDA 40
//#define I2C1_SCL 41 

//--------------LEDs----------------
#define LEDR_PIN 8
#define LEDV_PIN 9
#define LEDB_PIN 7

#define V_BAT_PIN 2
#define CAM_PIN 10
//#define BUZ_1 39
//#define BUZ_2 38

//-------------MOTEURS--------------
const int  MOTPIN_1 = 5;		// Moteur Avant Droit
const int  MOTPIN_2 = 6;		// Moteur Arrière Droit
const int  MOTPIN_3 = 3;		// Moteur Arriere Gauche
const int  MOTPIN_4 = 4; 	// Moteur Avant Gauche

// ################################################################
