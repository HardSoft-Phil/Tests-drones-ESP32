// JPC d'après: 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the RC receiver

#include "src/SBUS/sbus.h"

//float channelNeutral[RC_CHANNELS] = {NAN}; // first element NAN means not calibrated
//float channelMax[RC_CHANNELS];
float channelNeutral[] = {0, 883, 200, 972, 512, 512, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float channelMax[] = {1651, 1540, 1713, 1630, 1472, 1472, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, 22, 23, true);
/* SBUS data */
bfs::SbusData data;

//=============================================================
void setupRC() {
  /* Begin the SBUS communication */
  sbus_rx.Begin();
}

//=============================================================
void readRC() {
  data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    memcpy(channels, data.ch, sizeof(channels)); // copy channels data
    normalizeRC();
    controlsTime = t;
  }
}

//=============================================================
void normalizeRC() {
  //	if (isnan(channelNeutral[0])) return; // skip if not calibrated
  for (uint8_t i = 0; i < RC_CHANNELS; i++) {
    //		controls[i] = mapf(channels[i], channelNeutral[i], channelMax[i], 0, 1);
    controls[i] = mapf(channels[i], 200, 2000, 0, 1);
  }
}

//=============================================================
void calibrateRC() {
  Serial.println("Calibrate RC: move all sticks to maximum positions in 4 seconds");
  Serial.println("··o     ··o\n···     ···\n···     ···");
  delay(4000);
  for (int i = 0; i < 30; i++) readRC(); // ensure the values are updated
  for (int i = 0; i < RC_CHANNELS; i++) {
    channelMax[i] = channels[i];
  }
  Serial.println("Calibrate RC: move all sticks to neutral positions in 4 seconds");
  Serial.println("···     ···\n···     ·o·\n·o·     ···");
  delay(4000);
  for (int i = 0; i < 30; i++) readRC(); // ensure the values are updated
  for (int i = 0; i < RC_CHANNELS; i++) {
    channelNeutral[i] = channels[i];
  }
  printRCCal();
}

//=============================================================
void printRCCal() {
  printArray(channelNeutral, RC_CHANNELS);
  printArray(channelMax, RC_CHANNELS);
}

//=============================================================
