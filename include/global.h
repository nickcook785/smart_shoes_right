#ifndef GLOBALS_H
#define GLOBALS_H

#include <BLEDevice.h>
#include <HX711.h>

extern bool deviceConnected;
extern bool measureWeight;
extern int retryCount;
extern float calibration_factor;
extern HX711 scale;
extern BLEServer* pServer;
extern BLECharacteristic* pCharacteristic;
extern BLECharacteristic* pWriteCharacteristic;

#endif
