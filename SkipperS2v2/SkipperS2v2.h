#ifndef MBED_SKIPPERS2V2
#define MBED_SKIPPERS2V2

#include "mbed.h"

namespace s2v2
{
    static const PinName CH1 = PC_6;    // PWM3/1, UART6_TX
    static const PinName CH2 = PC_7;    // PWM3/2N, SPI2_SCLK, UART6_RX
    static const PinName CH3 = PB_0;    // PWM1/2N, SPI5_SCLK, ADC1/8
    static const PinName CH4 = PB_1;    // PWM1/3N, SPI5_SSEL, ADC1/9
    static const PinName CH5 = PB_6;    // PWM4/1, UART1_TX, I2C1_SCL
    static const PinName CH6 = PB_7;    // PWM4/2, UART1_RX, I2C1_SDA
    static const PinName CH7 = PB_8;    // PWM4/3, SPI5_MOSI, I2C1_SCL
    static const PinName CH8 = PB_9;    // PWM4/4, SPI2_SSEL, I2C1_SDA

    static const PinName LED1 = PA_0;
    static const PinName LED2 = PA_1;
    static const PinName LED3 = PB_4;
    static const PinName LED4 = PB_5;
    
    static const PinName SW2 = PC_14;
        
    static const PinName I2C_SDA = PC_9;    //PWM3/4, I2C3_SDA
    static const PinName I2C_SCL = PA_8;    //PWM1/1, I2C3_SCL

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