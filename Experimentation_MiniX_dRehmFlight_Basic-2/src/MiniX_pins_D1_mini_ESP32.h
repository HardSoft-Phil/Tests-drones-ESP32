// ################################################################
// #### * Carte WEMOS_D1_mini_ESP32  comprenant: 				               ####
// ####   -> processeur ESP32 Wroom 32                         								   ####
// ####   -> module MPU6050 ou MPU9250                         						   ####
// ####   -> 4 x moteurs courant continu (brushed)             							   ####
// #### * Télécommande par ESP_NOW ou radio (au choix)         				   ####
// ################################################################
// Révision du 12/02/2025

// BMP180    adr: 0x77
// MPU6550  adr : 0x68
// IMU9250   adr: 0x68

// Pins MPU6550 I2c :
const int SCL_PIN		= 22;
const int SDA_PIN	= 21;
const int INT_PIN		= 26;	// * INT commun avec MPU9250

// Pins MPU9250 SPI :
const int SCLK_PIN = 18;		// SCL MPU9250
const int MOSI_PIN = 23;		// SDA MPU9250
const int MISO_PIN = 19; 	// EDA MPU9250
const int CS_PIN       =  5;		// CS MPU9250

// Pins Leds :
const int LEDR_PIN   =  14;
const int LEDV_PIN   =  0;
//const int LEDB_PIN   = -1;	// non attribué
const int LED_PIN      =  2;	// BUILDIN

const int V_BAT_PIN  = 32;	
//const int CAM_PIN     = -1;	// non attribué

// Pins moteurs :
const int  MOTPIN_1 = 27;  	// Moteur Avant Droit
const int  MOTPIN_2 = 25; 	// Moteur Arrière Droit
const int  MOTPIN_3 = 12; 	// Moteur Arriere Gauche
const int  MOTPIN_4 = 4;		// Moteur Avant Gauche

// Pins radio :
const int SBUS_RX =   17; //22;		// Rx SBUS
const int SBUS_TX =    -1; //23;		// Tx SBUS bidon
//const int  CANAL_1 = 34;	// Throtlle
//const int  CANAL_2 = 35;	// Yaw
//const int  CANAL_3 = 36;	// Roll
//const int  CANAL_4 = 39;	// Pitch
//const int  CANAL_5 = 32;	// Aux1
//const int  CANAL_6 = 33;	// Aux2
