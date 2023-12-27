#define DEBUGLEVEL DEBUGLEVEL_DEBUGGING
#include <Arduino.h>
#include "MPU6050_light/MPU6050_light.h"
#include "adcObj/adcObj.hpp"
#include <ArduinoLog.h>


//#include "Debug/debug.h"


MPU6050 mpu(Wire);
adcObj damper1(ADC1_CHANNEL_4);
adcObj damper2(ADC1_CHANNEL_6);
adcObj damper3(ADC1_CHANNEL_7);
adcObj damper4(ADC1_CHANNEL_5);




void setup() {
  Serial.begin(9600);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  Wire.begin();

  byte status = mpu.begin();

   if(status!=0) {
    Log.error("MPU6050 not initialized - ERROR STATUS: %d", status);
   }

   
   else {
    Log.notice("MPU status: %d\n", status);
    Log.notice("Calculating offsets, do not move MPU6050\n");
    mpu.calcOffsets();
    Log.notice("Done - MPU initialized\n");
   }




  
}

void loop() {
  mpu.update();
  Log.verboseln("Ax:%F, Ay:%F, Az:%F, ACCx:%F, ACCy:%F, ACCz:%F", mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ(), mpu.getAccX(),mpu.getAccY(),mpu.getAccZ());
  int d1 = damper1.getVoltage();
  int d2 = damper2.getVoltage();
  int d3 = damper3.getVoltage();
  int d4 = damper4.getVoltage();
  Log.verboseln("\n D1: %d, D2: %d, D3: %d, D4: %d \n\n", d1, d2, d3, d4);
  //Log.verboseln("\n D1: %d\n\n", d1);


   delay(1000);
}
