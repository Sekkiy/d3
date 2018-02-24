#ifndef D3_CONFIG_H
#define D3_CONFIG_H


namespace DEFAULT{
    const uint16_t NTL_ELE = 1500;
    const uint16_t FWD_ELE = NTL_ELE+60;
    const uint16_t BCK_ELE = NTL_ELE-60;

    const uint16_t NTL_AIL = 1500;
    const uint16_t RGT_AIL = NTL_AIL+60;
    const uint16_t LFT_AIL = NTL_AIL-60;

    const uint16_t NTL_RUD = 1500;
    const uint16_t RGT_TURN_RUD  = NTL_RUD+60;
    const uint16_t LFT_TURN_RUD = NTL_RUD-60;

    const float PRESSURE_PGAIN = 0.1;
    const float PRESSURE_DGAIN = 0.0;
    const float TARGET_HEIGHT = 5.0;
    const float GOAL_LATITUDE = 35.700350;
    const float GOAL_LONGTITUDE = 139.517864;
    const float THLOTTOLE_HOVERING = 0.55;
    const float THRESHOLD_HEIGHT_RANGE = 0.33;
    const int16_t COMPASS_CALIBRATION_X = -13;
    const int16_t COMPASS_CALIBRATION_Y = -169;
    const int16_t COMPASS_CALIBRATION_Z = -208;
    
    const uint16_t TLIM_AIL = 0;
    const uint16_t TLIM_ELE = 0;
    const uint16_t TLIM_RUD = 0;
}


#endif  //D3_CONFIG_H