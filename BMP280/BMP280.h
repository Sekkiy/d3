/**
 *  BME280 Combined humidity and pressure sensor library
 *
 *  @author  Toyomasa Watarai
 *  @version 1.0
 *  @date    06-April-2015
 *
 *  Library for "BME280 temperature, humidity and pressure sensor module" from Switch Science
 *    https://www.switch-science.com/catalog/2236/
 *
 *  For more information about the BME280:
 *    http://ae-bst.resource.bosch.com/media/products/dokumente/bme280/BST-BME280_DS001-10.pdf
 */
 
#ifndef MBED_BMP280_H
#define MBED_BMP280_H

#include "mbed.h"

//#define DEFAULT_SLAVE_ADDRESS (0x77)
#define DEFAULT_SLAVE_ADDRESS (0x76)

//#define _DEBUG
#ifdef _DEBUG
extern Serial pc;
#define DEBUG_PRINT(...) pc.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif


/** BME280 class
 *
 *  BME280: A library to correct environmental data using Boshe BME280 device
 *
 *  BME280 is an environmental sensor
 *  @endcode
 */
 
class BMP280
{
public:
    enum OptionPowerMode{
        SLEEP = 0,
        FORCED,
        NOMAL = 3
    };
    enum OptionOverSampling{
        OS_SKIPPED = 0,
        OS_X1,
        OS_X2,
        OS_X4,
        OS_X8,
        OS_X16
    };
    enum OptionPressFilter{
        PF_OFF = 0,
        PF_X2,
        PF_X4,
        PF_X8,
        PF_X16
    };
    enum OptionStandByTime{
        SBT_0_5MS = 0,
        SBT_62_5MS,
        SBT_125MS,
        SBT_250MS,
        SBT_500MS,
        SBT_1000MS,
        SBT_2000MS,
        SBT_4000MS
    };
    enum FilterSelection{
        HANDHELD_DEVICE_LOW_POWER,
        HANDHELD_DEVICE_DYNAMIC,
        WETHER_MONITORING,
        FLOOR_CHANGE_DETECTION,
        DROP_DETECTION,
        INDOOR_NAVIGATION        
    };

    /** Create a BME280 instance
     *  which is connected to specified I2C pins with specified address
     *
     * @param sda I2C-bus SDA pin
     * @param scl I2C-bus SCL pin
     * @param slave_adr (option) I2C-bus address (default: 0x76)
     */
    BMP280(PinName sda, PinName sck, char slave_adr = DEFAULT_SLAVE_ADDRESS);

    /** Create a BME280 instance
     *  which is connected to specified I2C pins with specified address
     *
     * @param i2c_obj I2C object (instance)
     * @param slave_adr (option) I2C-bus address (default: 0x76)
     */
    BMP280(I2C &i2c_obj, char slave_adr = DEFAULT_SLAVE_ADDRESS);

    /** Destructor of BME280
     */
    virtual ~BMP280();

    /** Initializa BME280 sensor
     *
     *  Configure sensor setting and read parameters for calibration
     *
     */
    void initialize(void);
    void initialize(FilterSelection filter);
    void initialize(OptionPowerMode opPM,
                    OptionOverSampling opPOS,
                    OptionOverSampling opTOS,
                    OptionPressFilter opPF,
                    OptionStandByTime opSBT,
                    float putPutRate);

    /** Read the current temperature value (degree Celsius) from BME280 sensor
     *
     */
    float getTemperature(void);

    /** Read the current pressure value (hectopascal)from BME280 sensor
     *
     */
    float getPressure(bool skipMeasureTemp = false);

    /** Read the current humidity value (humidity %) from BME280 sensor
     *
     */
  //  float getHumidity(void);

    char getID();

    float getSampleRate();

    float getCycle_s();
    float getCycle_ms();
    float getCycle_us();

    void setPowerMode(OptionPowerMode op);
    void setPressOverSampling(OptionOverSampling op);
    void setTempOverSampling(OptionOverSampling op);
    void setPressFilter(OptionPressFilter op);
    void setStandByTime(OptionStandByTime op);

    void selectFilter(FilterSelection filter);

    void enableSPI3WriteMode(bool enable);
    void resetSettings();
    bool whoAmI();


private:

    I2C         *i2c_p;
    I2C         &i2c;
    char        address;
    uint16_t    dig_T1;
    int16_t     dig_T2, dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint16_t    dig_H1, dig_H3;
    int16_t     dig_H2, dig_H4, dig_H5, dig_H6;
    int32_t     t_fine;

    char pressOverSampling; //0,1,2,4,8,16
    char tempOverSampling; //0,1,2,4,8,16
    char powerMode; //sleep,forced.nomal
    char standByTime; //0.5,62.5,125,250,500,1000,2000,4000 [ms]
    char pressFilter; //off,2,4,8,16
    bool SPI3WriteMode; //off,on
    float sampleTime;
    float sampleRate;

    void reset();
    char ctrl_meas();
    char config();

    void setSetters(OptionPowerMode opPM,
                    OptionOverSampling opPOS,
                    OptionOverSampling opTOS,
                    OptionPressFilter opPF,
                    OptionStandByTime opSBT,
                    float sampleRate);
    void setOutputDataRate(float Hz);
};

inline float BMP280::getSampleRate(){return sampleRate;}

inline float BMP280::getCycle_s(){return sampleTime*0.001;}
inline float BMP280::getCycle_ms(){return sampleTime;}
inline float BMP280::getCycle_us(){return sampleTime*1000;}

inline void BMP280::setPressOverSampling(OptionOverSampling op){pressOverSampling = static_cast<char>(op);}
inline void BMP280::setTempOverSampling(OptionOverSampling op){tempOverSampling = static_cast<char>(op);}
inline void BMP280::setPowerMode(OptionPowerMode op){powerMode = static_cast<char>(op);}
inline void BMP280::setStandByTime(OptionStandByTime op){standByTime = static_cast<char>(op);}
inline void BMP280::setPressFilter(OptionPressFilter op){pressFilter = static_cast<char>(op);}
inline void BMP280::enableSPI3WriteMode(bool enable){SPI3WriteMode = enable;}

inline char BMP280::ctrl_meas()
{    
    return ((tempOverSampling << 5) | (pressOverSampling << 2) | powerMode);
}

inline char BMP280::config()
{
    return ((standByTime << 5) | (pressFilter << 2) | (static_cast<char>(SPI3WriteMode)));
}


inline void BMP280::setSetters(OptionPowerMode opPM, OptionOverSampling opPOS, OptionOverSampling opTOS, 
                               OptionPressFilter opPF, OptionStandByTime opSBT, float outputRate)
{
    setPowerMode(opPM);
    setPressOverSampling(opPOS);
    setTempOverSampling(opTOS);
    setPressFilter(opPF);
    setStandByTime(opSBT);
    setOutputDataRate(outputRate);
}

inline void BMP280::setOutputDataRate(float Hz){
    sampleRate = Hz;
    sampleTime = 1000.0 / Hz;
}


#endif // MBED_BME280_H
