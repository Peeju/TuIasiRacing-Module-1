#include <Arduino.h>
//#include "MPU6050/MPU6050.h"
#include "adcObj/adcObj.hpp"
#include <ArduinoLog.h>
#include "driver/can.h"
#include "aditional/aditional.hpp"
#include "MPU9250/MPU9250c.hpp"


#define TX_GPIO_NUM   GPIO_NUM_14
#define RX_GPIO_NUM   GPIO_NUM_27
#define LOG_LEVEL LOG_LEVEL_VERBOSE


MPU9250c MPU;


u_int16_t status;



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

twai_message_t tx_msg_mpu;
twai_message_t tx_msg_damp;
twai_message_t tx_msg_acc;
twai_message_t tx_msg_gyro;

void setup() {
  Serial.begin(9600);
  Log.begin(LOG_LEVEL, &Serial);
  MPU.begin();
 
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

    
  tx_msg_mpu.data_length_code=8;
  tx_msg_mpu.identifier=0x113;
  tx_msg_mpu.flags=CAN_MSG_FLAG_NONE;

  tx_msg_damp.data_length_code=8;
  tx_msg_damp.identifier=0x112;
  tx_msg_damp.flags=CAN_MSG_FLAG_NONE;

  tx_msg_acc.data_length_code=6;
  tx_msg_acc.identifier=0x111;
  tx_msg_acc.flags=CAN_MSG_FLAG_NONE;

  tx_msg_gyro.data_length_code=6;
  tx_msg_gyro.identifier=0x107;
  tx_msg_gyro.flags=CAN_MSG_FLAG_NONE;
}


void loop() {
  //Log.notice("\n\n********New loop started********\n");

  
  
  MPU.read();

  #if LOG_LEVEL==LOG_LEVEL_VERBOSE
  
  int d1 = damper1.getVoltage();
  int d2 = damper2.getVoltage();
  int d3 = damper3.getVoltage();
  int d4 = damper4.getVoltage();
  //Log.verboseln("D1: %d, D2: %d, D3: %d, D4: %d", d1, d2, d3, d4);
  
 
  tx_msg_damp.data[0]=d1/100;
  tx_msg_damp.data[1]=d1%100;
  tx_msg_damp.data[2]=d2/100;
  tx_msg_damp.data[3]=d2%100;
  tx_msg_damp.data[4]=d3/100;
  tx_msg_damp.data[5]=d3%100;
  tx_msg_damp.data[6]=d4/100;
  tx_msg_damp.data[7]=d4%100;

  float roll=MPU.getRoll();
  float pitch = MPU.getPitch();
  float yaw = MPU.getYaw();
  // Serial.print("Roll:   ");
  // Serial.print(roll);
  // Serial.print("  Pitch: ");
  // Serial.print(pitch);
  // Serial.print("  Yaw: ");
  // Serial.println(yaw);
  tx_msg_mpu.data[6]=0;
  if(roll >= 0) {
    convert(roll, tx_msg_mpu.data);

  }
  else {
    convert(roll*-1, tx_msg_mpu.data);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+1; 
  }

  if(pitch >= 0) {
    convert(pitch, tx_msg_mpu.data+2);
  }
  else {
    convert(pitch*-1, tx_msg_mpu.data+2);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+2; 
  }

  if(yaw >= 0) {
    convert(yaw, tx_msg_mpu.data+4);
  }
  else {
    convert(yaw*-1, tx_msg_mpu.data+4);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+4; 
  }

  //todo get pentru acceleratii si gyro
  float ax = MPU.currentData.accelX;
  float ay = MPU.currentData.accelY; 
  float az = MPU.currentData.accelZ;
  convert(ax, tx_msg_acc.data);
  convert(ay, tx_msg_acc.data+2);
  convert(az, tx_msg_acc.data+4);

  float gx = MPU.currentData.gyroX;
  float gy = MPU.currentData.gyroY; 
  float gz = MPU.currentData.gyroZ;
  convert(gx, tx_msg_gyro.data);
  convert(gy, tx_msg_gyro.data+2);
  convert(gz, tx_msg_gyro.data+4);
  
  #else
  
  float KalmanD1 = damper1.KalmanVoltage();
  float KalmanD2 = damper2.KalmanVoltage();
  float KalmanD3 = damper3.KalmanVoltage();
  float KalmanD4 = damper4.KalmanVoltage();
    
  tx_msg_damp.data[0]=int(KalmanD1)/100;
  tx_msg_damp.data[1]=int(KalmanD1)%100;
  tx_msg_damp.data[2]=int(KalmanD2)/100;
  tx_msg_damp.data[3]=int(KalmanD2)%100;
  tx_msg_damp.data[4]=int(KalmanD3)/100;
  tx_msg_damp.data[5]=int(KalmanD3)%100;
  tx_msg_damp.data[6]=int(KalmanD4)/100;
  tx_msg_damp.data[7]=int(KalmanD4)%100;
  

  float roll = MPU.getRoll();
  float pitch = MPU.getPitch();
  float yaw = MPU.getYaw();


  // //* Function covert() bitshifts the values and puts them into 2 * 8 bits. 
  // //* The value is checked if it is negative and if it is, it is converted to positive and in the last byte of the message the bit 
  // //* that corresponds to its position gets converted to one
  // //* Example: if (second value is negative) then 0000+2=0010;

 

  tx_msg_mpu.data[6]=0;
  //roll
  if(roll >= 0) {
    convert(roll, tx_msg_mpu.data);

  }
  else {
    convert(roll*-1, tx_msg_mpu.data);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+1; 
  }
  //pitch
  if(pitch >= 0) {
    convert(pitch, tx_msg_mpu.data+2);
  }
  else {
    convert(pitch*-1, tx_msg_mpu.data+2);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+2; 
  }
  //yaw
  if(yaw >= 0) {
    convert(yaw, tx_msg_mpu.data+4);
  }
  else {
    convert(yaw*-1, tx_msg_mpu.data+4);
    tx_msg_mpu.data[6]= tx_msg_mpu.data[6]+4; 
  }

  //accel
  convert(MPU.currentData.accelX, tx_msg_acc.data);
  convert(MPU.currentData.accelY, tx_msg_acc.data+2);
  convert(MPU.currentData.accelZ, tx_msg_acc.data+4);
 
  #endif
  
  
  status = can_transmit(&tx_msg_mpu, pdMS_TO_TICKS(1000));
   if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.error("Can driver restarted");
  }

  status = can_transmit(&tx_msg_acc, pdMS_TO_TICKS(1000));
   if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.error("Can driver restarted");
  }

  status = can_transmit(&tx_msg_gyro, pdMS_TO_TICKS(1000));
   if(status==ESP_OK) {
    Log.noticeln("Can message sent");
  }
  else {
    Log.errorln("Can message sending failed with error code: %s ;\nRestarting CAN driver", esp_err_to_name(status));
    can_stop();
    can_driver_uninstall();
    can_driver_install(&g_config, &t_config, &f_config);
    status = can_start();
    if(status==ESP_OK) Log.error("Can driver restarted");
  }
  
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
  
}