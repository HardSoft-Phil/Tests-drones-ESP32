// BMP180  I2c adr: 0x77
// MPU6550 I2c adr: 0x68
// IMU9250 I2c adr: 0x68

//##########################################################
//REQUIRED LIBRARIES (included with download in main sketch folder)

//#include <EEPROM.h>   // Gestion eeprom pour paramètres
#include "src/MiniX_pins_D1_mini_ESP32.h"

//#define ONESHOT // Ca ne veut pas marcher, grr

//#ifdef ONESHOT
//#include "OneShot.h"
//#else
//#include "Moteurs1.h"
//#endif

//##########################################################
//                      MPU9250 DEFINES
//##########################################################

//Uncomment only one IMU
//#define USE_MPU6050_I2C
#define USE_MPU9250_SPI //default

//Uncomment only one full scale gyro range (deg/sec)
//#define GYRO_250DPS //default
#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

#if defined USE_MPU6050_I2C
#include <Wire.h>     // I2c communication
#include "src/MPU6050/MPU6050.h"
MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
#include "src/MPU9250/MPU9250.h"
#include <SPI.h>      // SPI communication
MPU9250 mpu9250(SPI, 5);
#else
#error No MPU defined...
#endif

//##########################################################
//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
#define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;

float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.20;
float AccErrorY = 2.00;
float AccErrorZ = -0.14;
float GyroErrorX = 0.23;
float GyroErrorY = 0.82;
float GyroErrorZ = 0.99;

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 1.52;
float MagErrorY = 13.41;
float MagErrorZ = -26.98;
float MagScaleX = 0.96;
float MagScaleY = 1.05;
float MagScaleZ = 0.99;

//##########################################################
//                 USER-SPECIFIED VARIABLES
//##########################################################

#define DEBUG

//Controller parameters (take note of defaults before modifying!):
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

uint32_t RledTimer = 0;
bool RblinkState = false;
uint32_t VledTimer = 0;
bool VblinkState = false;
uint32_t ledTimer = 0;
bool blinkState = false;

//--------------RADIO---------------
#define CHANNELS 8
unsigned long rcValue[CHANNELS];  // en µs, centre = 1500
// Ordre des voies du message de téléréception
#define ROL 0
#define PIT 1
#define THR 2
#define RUD 3
#define AUX1 4
#define AUX2 5

// Radio PWM Values
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINTHRUST 1150

//-------------MOTEURS---------------
enum mot { AVD, ARD, ARG, AVG };
float servo[4];

//=============================================================
  
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
float dt;
unsigned long current_time, prev_time;

uint32_t  tnow; 
uint32_t LoopTimer;
uint32_t t1Hz;
float t=0.003;      //time cycle

extern int16_t accZero[3];
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;

enum ang { ROLL,PITCH,YAW };

static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t magADC[3];
int16_t magXcoef, magYcoef, magZcoef;
static int16_t gyroData[3];
static float angle[3] = {0,0,0};  
extern int calibratingA;

static int16_t rcCommand[] = {0,0,0};
static int16_t axisPID[3];

#define ACCRO     0
#define STABLE    1
byte modeVol = STABLE; // variable de type de vol du drone
static int8_t oldmode;

#define ARRET   0   // Mode ARRET
#define ARME    1   // Mode ARME
#define MARCHE  2   // Mode MARCHE
byte etat = ARRET;      // variable de l'état de fonctionnement du drone

bool camMode = 0;
bool stableMode = 0;

boolean armed = false;
uint8_t armct = 0;

int debugvalue = 0;
