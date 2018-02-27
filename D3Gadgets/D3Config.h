#ifndef D3_CONFIG_H
#define D3_CONFIG_H

namespace DEFAULT{
    const uint16_t NTL_AIL = 1567;
    const uint16_t AIL_INCLIMENTAL = 60;
    const uint16_t RGT_AIL = NTL_AIL + AIL_INCLIMENTAL;
    const uint16_t LFT_AIL = NTL_AIL - AIL_INCLIMENTAL;

    const uint16_t NTL_ELE = 1476;
    const uint16_t ELE_INCLIMENTAL = 60;
    const uint16_t FWD_ELE = NTL_ELE + ELE_INCLIMENTAL;
    const uint16_t BCK_ELE = NTL_ELE - ELE_INCLIMENTAL;

    const uint16_t NTL_RUD = 1547;
    const uint16_t RUD_INCLIMENTAL = 60;    
    const uint16_t RGT_TURN_RUD = NTL_RUD + RUD_INCLIMENTAL;
    const uint16_t LFT_TURN_RUD = NTL_RUD - RUD_INCLIMENTAL;

    const float PRESSURE_PGAIN = 0.009;
    const float PRESSURE_IGAIN = 0.0;
    const float PRESSURE_DGAIN = 0.008;
    const float TARGET_HEIGHT = 5.0;
    const float GOAL_LATITUDE = 35.700350;
    const float GOAL_LONGTITUDE = 139.517864;
    const float THLOTTOLE_HOVERING = 0.55;
    const float THRESHOLD_HEIGHT_RANGE = 0.33;
    const float START_TURN_DEGREE = 80.0;
    const float MAX_STOP_SECOND = 0.8;
    const float GAIN_V_TO_S = MAX_STOP_SECOND / 15.0;
    const uint8_t COMPASS_AXIS_FORWARD = 1; //D3GuideのcompassAxisの整数値と同じ
    const uint8_t COMPASS_AXIS_RIGHT = 0;
    const float CHANGE_SEQUENCE_DISTANCE = 3.5;

    const int16_t COMPASS_CALIBRATION_X = -250;
    const int16_t COMPASS_CALIBRATION_Y = -270;
    const int16_t COMPASS_CALIBRATION_Z = -208;

    const uint16_t TLIM_AIL = 0;
    const uint16_t TLIM_ELE = 0;
    const uint16_t TLIM_RUD = 0;
}


#endif  //D3_CONFIG_H