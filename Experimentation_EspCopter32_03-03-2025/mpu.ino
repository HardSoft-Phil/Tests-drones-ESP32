
//####################################################
void IMUinit() {
  //DESCRIPTION: Initialise IMU connection I2C

  #if defined USE_MPU6050_I2C
//    Wire.begin();
  Wire.begin(I2C0_SDA, I2C0_SCL); // initialisation I2c pour ESP32-S2
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
//    Wire.setClock(400000);
    
    mpu6050.initialize();
    
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialisation échouée");
//      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    
  #elif defined USE_MPU9250_SPI
    int status = mpu9250.begin();    

    if (status < 0) {
#ifdef DEBUG
      Serial.println("MPU9250 initialisation échouée");
      Serial.print(" - Status: ");
      Serial.println(status);
#endif      
      while(1) {}
    }

// À partir de l'état de réinitialisation, tous les registres doivent 
// être à 0x00, nous devrions donc être à la fréquence d'échantillonnage
// maximale avec le(s) filtre(s) passe-bas numérique(s) désactivé(s). 
// Il suffit de définir les plages de pleine échelle souhaitées    
    mpu9250.setGyroRange(GYRO_SCALE);
    mpu9250.setAccelRange(ACCEL_SCALE);
    mpu9250.setMagCalX(MagErrorX, MagScaleX);
    mpu9250.setMagCalY(MagErrorY, MagScaleY);
    mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
    // Règle le gyro et l'acc à 1 kHz, le magnétomètre à 100 Hz
    mpu9250.setSrd(0);
  #endif
}

//####################################################
void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  //DESCRIPTION : Demande l'ensemble complet de données à partir des données du gyroscope
  //    et de l'accéléromètre à filtre IMU et LP
  //    Lit les données de l'accéléromètre et du gyroscope de l'IMU sous la forme AccX,
  //    AccY, AccZ, GyroX, GyroY, GyroZ.
  //    Ces valeurs sont mises à l'échelle conformément à la fiche technique de l'IMU
  //    pour les mettre dans les unités correctes de g et de rad/sec. Un simple filtre
  //    passe-bas de premier ordre est utilisé pour éliminer le bruit haute fréquence
  //    dans ces signaux bruts. En général, vous souhaitez couper tout ce qui dépasse
  //    80 Hz, mais si votre taux de boucle n'est pas assez rapide, le filtre passe-bas
  //    provoquera un décalage dans les lectures. Les paramètres de filtre B_gyro et
  //    B_accel sont définis pour être bons pour un taux de boucle de 2 kHz.
  //    Enfin, les erreurs constantes trouvées dans calculate_IMU_error() au démarrage
  //    sont soustraites des lectures.
  
//  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  #if defined USE_MPU6050_I2C
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  #endif

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  accADC[0] = AccX;
  accADC[1] = AccY;
  accADC[2] = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  gyroADC[0] = GyroX;
  gyroADC[1] = GyroY;
  gyroADC[2] = GyroZ;
  
  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;

  magADC[0] = MagX;
  magADC[1] = MagY;
  magADC[2] = MagZ;
}

//####################################################
void calculate_IMU_error() {
  // DESCRIPTION : calcule l'erreur IMU au démarrage.
  // Remarque : le véhicule doit être mis sous tension sur une surface plane.
  //  Ne vous inquiétez pas trop de ce que cela fait. Les valeurs d'erreur
  //  qu'il calcule sont appliquées aux valeurs brutes du gyroscope et de
  //  l'accéléromètre AccX, AccY, AccZ, GyroX, GyroY, GyroZ dans getIMUdata().
  //  Cela élimine la dérive dans la mesure.
  
//  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  
  //Read IMU values 1000 times
  int c = 0;
  while (c < 1000) {
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;
}

//####################################################
void calibrateMagnetometer() { // Le magnétomètre AK8963 est inclu dans le MPU9250
  #if defined USE_MPU9250_SPI 
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = mpu9250.calibrateMag();
    if(success) {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
      Serial.print("float MagErrorX = ");
      Serial.print(mpu9250.getMagBiasX_uT());
      Serial.println(";");
      Serial.print("float MagErrorY = ");
      Serial.print(mpu9250.getMagBiasY_uT());
      Serial.println(";");
      Serial.print("float MagErrorZ = ");
      Serial.print(mpu9250.getMagBiasZ_uT());
      Serial.println(";");
      Serial.print("float MagScaleX = ");
      Serial.print(mpu9250.getMagScaleFactorX());
      Serial.println(";");
      Serial.print("float MagScaleY = ");
      Serial.print(mpu9250.getMagScaleFactorY());
      Serial.println(";");
      Serial.print("float MagScaleZ = ");
      Serial.print(mpu9250.getMagScaleFactorZ());
      Serial.println(";");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }
  
    while(1); //halt code so it won't enter main loop until this function commented out
  #endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); //halt code so it won't enter main loop until this function commented out
}

//####################################################
