#include <Arduino.h>
#include "MPU6050_light/MPU6050_light.h"
#include "adcObj/adcObj.hpp"
#include <ArduinoLog.h>
#include "driver/can.h"

#define TX_GPIO_NUM   GPIO_NUM_14
#define RX_GPIO_NUM   GPIO_NUM_27
#define LOG_LEVEL LOG_LEVEL_VERBOSE

//todo: convert MPU values from double to int
//!debug ADC conversion to can



byte status;


MPU6050 mpu(Wire);
adcObj damper1(ADC1_CHANNEL_4);
adcObj damper2(ADC1_CHANNEL_4);
adcObj damper3(ADC1_CHANNEL_4);
adcObj damper4(ADC1_CHANNEL_4);

static const can_general_config_t g_config = {.mode = TWAI_MODE_NO_ACK, .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,        
                                                                    .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      
                                                                    .tx_queue_len = 1, .rx_queue_len = 5,                           
                                                                    .alerts_enabled = TWAI_ALERT_ALL,  .clkout_divider = 0,        
                                                                    .intr_flags = ESP_INTR_FLAG_LEVEL1};
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

  

void setup() {
  Serial.begin(9600);
  Log.begin(LOG_LEVEL, &Serial);

  Wire.begin();

  status = mpu.begin();

   if(status!=0) {
    Log.error("MPU6050 not initialized - ERROR STATUS: %d", status);
   }
   else {
    Log.notice("MPU status: %d\n", status);
    Log.notice("Calculating offsets, do not move MPU6050\n");
    mpu.calcOffsets();
    Log.notice("Done - MPU initialized\n");
   }

    status = can_driver_install(&g_config, &t_config, &f_config);
    if(status==ESP_OK){
      Log.noticeln("Can driver installed");
    }
    else {
      Log.errorln("Can driver installation failed with error: %s", esp_err_to_name(status));
    }

    status=can_start();
    if(status==ESP_OK){
      Log.noticeln("Can started");
    }
    else {
      Log.errorln("Can starting procedure failed with error: %s", esp_err_to_name(status));
    }
}


void loop() {
  twai_message_t tx_msg_mpu;
  tx_msg_mpu.data_length_code=6;
  tx_msg_mpu.identifier=0x111;
  tx_msg_mpu.flags=CAN_MSG_FLAG_NONE;

  twai_message_t tx_msg_damp;
  tx_msg_damp.data_length_code=4;
  tx_msg_damp.identifier=0x112;
  tx_msg_damp.flags=CAN_MSG_FLAG_NONE;
  
  mpu.update();

  #ifdef LOG_LEVEL_VERBOSE
  int aX=mpu.getAngleX();
  int aY=mpu.getAngleY();
  int aZ=mpu.getAngleZ();
  int acX=mpu.getAccX();
  int acY=mpu.getAccY();
  int acZ=mpu.getAccZ();
  Log.verboseln("Ax:%F, Ay:%F, Az:%F, ACCx:%F, ACCy:%F, ACCz:%F", aX, aY, aZ, acX, acY, acZ);
  
  int d1 = damper1.getVoltage();
  // int d2 = damper2.getVoltage();
  // int d3 = damper3.getVoltage();
  // int d4 = damper4.getVoltage();
  Log.verboseln("\n D1: %d, D2: %d, D3: %d, D4: %d \n\n", d1, d1, d1, d1);
  //Log.verboseln("\n D1: %d\n\n", d1);
  
 
  tx_msg_damp.data[0]=d1;
  tx_msg_damp.data[1]=d1;
  tx_msg_damp.data[2]=d1;
  tx_msg_damp.data[3]=d1;

  tx_msg_mpu.data[0]=aX;
  tx_msg_mpu.data[1]=aY;
  tx_msg_mpu.data[2]=aZ;
  tx_msg_mpu.data[3]=acX;
  tx_msg_mpu.data[4]=acY;
  tx_msg_mpu.data[5]=acZ;
  #else
  tx_msg_damp.data[0]=damper1.getVoltage();
  tx_msg_damp.data[1]=damper2.getVoltage();
  tx_msg_damp.data[2]=damper3.getVoltage();
  tx_msg_damp.data[3]=damper4.getVoltage();

  tx_msg_mpu.data[0]=mpu.getAngleX();
  tx_msg_mpu.data[1]=mpu.getAngleY();
  tx_msg_mpu.data[2]=mpu.getAngleZ();
  tx_msg_mpu.data[3]=mpu.getAccX();
  tx_msg_mpu.data[4]=mpu.getAccY();
  tx_msg_mpu.data[5]=mpu.getAccZ();
  #endif
  
  status = can_transmit(&tx_msg_damp, pdMS_TO_TICKS(1000));
  if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.errorln("Can driver restarted");
  }

  
  status = can_transmit(&tx_msg_mpu, portMAX_DELAY);
   if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.errorln("Can driver restarted");
  }

  
}
