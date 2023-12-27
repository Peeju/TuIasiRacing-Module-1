#include "adcObj.hpp"

adcObj::adcObj(adc1_channel_t tempChannel, adc_unit_t tempUnit, adc_atten_t tempAtten, adc_bits_width_t tempBits) :
 channel(tempChannel), unit(tempUnit), atten(tempAtten), bits(tempBits) {
     esp_adc_cal_characterize(unit, atten, bits, 0, &adc_chars);
     esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
     ESP_ERROR_CHECK(adc1_config_width(bits));
     ESP_ERROR_CHECK(adc1_config_channel_atten(channel, atten));
    
 }  

int adcObj::getVoltage() {
    int voltage = adc1_get_raw(channel);
    return esp_adc_cal_raw_to_voltage(adc1_get_raw(channel), &adc_chars);
}
