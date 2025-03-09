//####################################################
void zeroPID() {
  //DESCRIPTION: Remise à zéro des paramètres PID
  error_roll = 0;
  error_pitch = 0;
  error_yaw = 0;
}

//####################################################
void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  //     Contrôle PID de base pour stabiliser le point de consigne d'angle en fonction
  //     des états souhaités roll_des, pitch_des et yaw_des calculés dans getDesState().
  //     L'erreur est simplement l'état souhaité moins l'état réel (ex. roll_des - roll_IMU).
  //     Deux fonctionnalités de sécurité sont implémentées ici concernant les termes I.
  //     Les termes I sont saturés dans des limites spécifiées au démarrage pour éviter
  //     une accumulation excessive. Cela peut être vu en maintenant le véhicule à un
  //     angle et en voyant les moteurs monter en puissance d'un côté jusqu'à ce qu'ils
  //     aient atteint le maximum de la manette des gaz... saturer I à une limite spécifiée
  //     corrige ce problème. La deuxième fonctionnalité définit par défaut les termes I à 0
  //     si la manette des gaz est au réglage minimum. Cela signifie que les moteurs ne
  //     commenceront pas à accélérer au sol et que les termes I démarreront toujours
  //     à partir de 0 au décollage. Cette fonction met à jour les variables roll_PID,
  //     pitch_PID et yaw_PID qui peuvent être considérées comme des signaux stabilisés 1D.
  //     Ils sont mélangés à la configuration du véhicule dans controlMixer().

  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch); //scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

//####################################################
void controlRATE() {
  // DESCRIPTION: Calcule les commandes de contrôle en fonction de l'erreur d'état
  //    Voir l'explication de controlANGLE().
  //    Tout est identique ici, sauf que l'erreur correspond désormais au taux souhaité - lecture brute du gyroscope.

  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID = .01 * (Kp_roll_rate * error_roll + Ki_roll_rate * integral_roll + Kd_roll_rate * derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID = .01 * (Kp_pitch_rate * error_pitch + Ki_pitch_rate * integral_pitch + Kd_pitch_rate * derivative_pitch); //scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

//####################################################
