#include "aditional.hpp"


void convert(int16_t x, uint8_t* data) {
    data[0] = (uint8_t)(x >> 0);
    data[1] = (uint8_t)(x >> 8);
}