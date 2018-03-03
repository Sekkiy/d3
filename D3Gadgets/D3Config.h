#ifndef D3_CONFIG_H
#define D3_CONFIG_H

namespace DEFAULT{
    const uint16_t NTL_AIL = 1535;
    const uint16_t AIL_INCLIMENTAL = 80;
    const uint16_t SLOW_AIL_INCLIMENTAL = 50;
    const uint16_t RGT_AIL = NTL_AIL + AIL_INCLIMENTAL;
    const uint16_t LFT_AIL = NTL_AIL - AIL_INCLIMENTAL;

    const uint16_t NTL_ELE = 1480;
    const uint16_t ELE_INCLIMENTAL = 80;
    const uint16_t SLOW_ELE_INCLIMENTAL = 50;
    const uint16_t FWD_ELE = NTL_ELE + ELE_INCLIMENTAL;
    const uint16_t BCK_ELE = NTL_ELE - ELE_INCLIMENTAL;

    const uint16_t NTL_RUD = 1515;
    const uint16_t RUD_INCLIMENTAL = 80;
    const uint16_t RGT_TURN_RUD = NTL_RUD + RUD_INCLIMENTAL;
    const uint16_t LFT_TURN_RUD = NTL_RUD - RUD_INCLIMENTAL;

    const float PRESSURE_PGAIN = 0.009;
    const float PRESSURE_IGAIN = 0.0;
    const float PRESSURE_DGAIN = 0.008;
    const float TARGET_HEIGHT = 4.5;
    const float GOAL_LATITUDE = 35.700350;
    const float GOAL_LONGTITUDE = 139.517864;
    const float THLOTTOLE_HOVERING = 0.33;
    const float THRESHOLD_HEIGHT_RANGE = 0.25;
    const float START_TURN_DEGREE = 50.0;
    const float MAX_STOP_SECOND = 0.8;
    const float GAIN_V_TO_S = MAX_STOP_SECOND / 15.0;
    const uint8_t COMPASS_AXIS_FORWARD = 4; //D3GuideのCompassAxisの整数値と同じ
    const uint8_t COMPASS_AXIS_RIGHT = 5;
    const float CHANGE_SEQUENCE_DISTANCE = 3.5;
    const float START_DECELERATION_DISTANCE = 25.0; 
    const float DECELERATION_RATE = 0.8;
    const float FINISH_LENGTH = 30;
    const uint8_t CAMERA_DIRECTION = 1; //D3GuideのCameraDirectionの整数値と同じ

    const int16_t COMPASS_CALIBRATION_X = -80;
    const int16_t COMPASS_CALIBRATION_Y = -92;
    const int16_t COMPASS_CALIBRATION_Z = -20;

    const uint16_t TLIM_AIL = 0;
    const uint16_t TLIM_ELE = 0;
    const uint16_t TLIM_RUD = 0;
}


#endif  //D3_CONFIG_H