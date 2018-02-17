#ifndef MBED_SKIPPERS2V2
#define MBED_SKIPPERS2V2

#include "mbed.h"

namespace s2v2
{
    static const PinName CH1 = PC_6;
    static const PinName CH2 = PC_7;
    static const PinName CH3 = PB_0;
    static const PinName CH4 = PB_1;
    static const PinName CH5 = PB_6;
    static const PinName CH6 = PB_7;
    static const PinName CH7 = PB_8;
    static const PinName CH8 = PB_9;

    static const PinName LED1 = PA_0;
    static const PinName LED2 = PA_1;
    static const PinName LED3 = PB_4;
    static const PinName LED4 = PB_5;
    
    static const PinName SW2 = PC_14;
        
    static const PinName I2C_SDA = PC_9;
    static const PinName I2C_SCL = PA_8;

    static const PinName MPU_SDA = PC_9;
    static const PinName MPU_SCL = PA_8;
    static const PinName MPU_INT = PC_0;

    static const PinName BMP_SDA = PC_9;
    static const PinName BMP_SCL = PA_8;

    static const PinName PC_TX = PA_2;
    static const PinName PC_RX = PA_3;
    
    static const PinName SBUS_TX = PA_9;
    static const PinName SBUS_RX = PA_10;

    static const PinName SD_DI = PB_15;     //MOSI
    static const PinName SD_DO = PB_14;     //MISO
    static const PinName SD_CLK = PB_13;    //SCLK
    static const PinName SD_CS = PB_12;
}

#endif // MBED_SKIPPERS2V2