
typedef union int16_ty
{
  int16_t d;
  byte    b[2];
};
typedef union float_ty
{
  float d;
  byte  b[4];
};

//=============================================================
void write_int16(int pos, int16_t d)
{
  int16_ty loc;
  loc.d = d;
  EEPROM.write(pos++,loc.b[0]);
  EEPROM.write(pos++,loc.b[1]);
}

//=============================================================
int16_t read_int16(int pos)
{
  int16_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  return loc.d;
}

//=============================================================
void write_float(int pos, float d)
{
  float_ty loc;
  loc.d = d;
  EEPROM.write(pos++,loc.b[0]);
  EEPROM.write(pos++,loc.b[1]);
  EEPROM.write(pos++,loc.b[2]);
  EEPROM.write(pos++,loc.b[3]);
}

//=============================================================
float read_float(int pos)
{
  float_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  loc.b[2] = EEPROM.read(pos++);
  loc.b[3] = EEPROM.read(pos++);
  return loc.d;
}

//=============================================================
void MPU_Read()  // Lecture paramètres d'étalonnage
{ // Accelerometre
  AccErrorX = read_float(0);
  AccErrorY = read_float(4);
  AccErrorZ = read_float(8);
  // Gyroscope
  GyroErrorX = read_float(12);
  GyroErrorY = read_float(16);
  GyroErrorZ = read_float(20);
  // Magnetometre
  MagErrorX = read_float(24);
  MagErrorY = read_float(28);
  MagErrorZ = read_float(32);
  MagScaleX = read_float(36);
  MagScaleY = read_float(40);
  MagScaleZ = read_float(44);
}

//=============================================================
void MPU_Store() // Sauvegarde paramètres d'étalonnage
{ // Accelerometre
  write_float(0, AccErrorX);
  write_float(4, AccErrorY);
  write_float(8, AccErrorZ);
  // Gyroscope
  write_float(12, GyroErrorX);
  write_float(16, GyroErrorY);
  write_float(20, GyroErrorZ);
  // Magnetometre
  write_float(24, MagErrorX);
  write_float(28, MagErrorY);
  write_float(32, MagErrorZ);
  write_float(36, MagScaleX);
  write_float(40, MagScaleY);
  write_float(44, MagScaleZ);
  EEPROM.write(116, 0xAB);
  EEPROM.commit();
}

//=============================================================
void PID_Read() // Lecture paramètres pid
{
  Kp_roll_angle  = read_float(48);
  Ki_roll_angle  = read_float(52);
  Kd_roll_angle  = read_float(56);
  B_loop_roll    = read_float(60);
  Kp_pitch_angle = read_float(64);
  Ki_pitch_angle = read_float(68);
  Kd_pitch_angle = read_float(72);
  B_loop_pitch   = read_float(76);
  Kp_roll_rate   = read_float(80);
  Ki_roll_rate   = read_float(84);
  Kd_roll_rate   = read_float(88);
  Kp_pitch_rate  = read_float(92);
  Ki_pitch_rate  = read_float(96);
  Kd_pitch_rate  = read_float(100);
  Kp_yaw         = read_float(104);
  Ki_yaw         = read_float(108);
  Kd_yaw         = read_float(112);
}

//=============================================================
void PID_Store() // Sauvegarde paramètres pid
{
  write_float(48,Kp_roll_angle);
  write_float(52,Ki_roll_angle);
  write_float(56,Kd_roll_angle);
  write_float(60,B_loop_roll);
  write_float(64,Kp_pitch_angle);
  write_float(68,Ki_pitch_angle);
  write_float(72,Kd_pitch_angle);
  write_float(76,B_loop_pitch);
  write_float(80,Kp_roll_rate);
  write_float(84,Ki_roll_rate);
  write_float(88,Kd_roll_rate);
  write_float(92,Kp_pitch_rate);
  write_float(96,Ki_pitch_rate);
  write_float(100,Kd_pitch_rate);
  write_float(104,Kp_yaw);
  write_float(108,Ki_yaw);
  write_float(112,Kd_yaw);
  EEPROM.write(115, 0xCD);
  EEPROM.commit();
}

//=============================================================
