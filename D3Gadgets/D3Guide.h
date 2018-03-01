#ifndef D3_GUIDE_H
#define D3_GUIDE_H

#include <string>
#include <vector>
#include "mbed.h"

class D3Guide
{
public:
    D3Guide(){}

    enum compassAxis{
        AXIS_X = 0,
        AXIS_Y,
        AXIS_Z,
        AXIS_mX,    //-X
        AXIS_mY,    //-Y
        AXIS_mZ     //-Z
    };


    void updateCurrentLocation(double lat, double lng);
    void updateCompass(int16_t magxyz[3]);
    void updateInputs(double lat, double lng, int16_t magxyz[3]);
    void setGoalLocation(double lat, double lng);
    void setMagBias(int16_t xbias, int16_t ybias, int16_t zbias);
    void setMagBias(int16_t xyzbias[3]);
    void setLocationError(double error);
    void setCompassAxis(compassAxis forward, compassAxis right);
    double getCompassRad();
    double getTurnRad();
    double getGoalDistance();
    bool divideVelocity(double val[2], double velocity, bool checkPrecision = true);
    
    void encode(char c);
    int16_t getRaspiX();
    int16_t getRaspiY();
    bool RaspiIsUpdated();

private:
    double mCurrentLat[2];
    double mCurrentLng[2];
    double mGoalLat;
    double mGoalLng;
    double mCompassRad;
    double mMagDeclination;
    double mLocationError;
    int16_t mMagBiasxyz[3];
    compassAxis mForward;
    compassAxis mRight;
    
    std::string mStr;
    bool mRaspiIsUpdated;
    int16_t mRaspiData[2];

    enum Condition{
        Stop
    } condition;

    constexpr static double DEGREE_TO_RADIAN = 0.017453292519943295769236907684886;
    constexpr static size_t MAX_RASPI_SENTENCE = 11 - 2;  //全出力数からスタートビットとストップビットを除いた数 

    double calcCompassRad(int16_t magxyz[3]);    
    double calcGpsRad(double lat1, double long1, double lat2, double long2);
    int16_t toFrameAxis(int16_t magxyz[3], compassAxis axis);
    double calcMagDeclination(double lat, double lng);
    double distanceBetween(double lat1, double long1, double lat2, double long2);
    double withinMPiToPi(double rad);
    double toRadian(double deg);

    bool decode(std::string& str);
};

static std::vector<std::string> split(const std::string& input, char delimiter);



inline void D3Guide::updateInputs(double lat, double lng, int16_t magxyz[3]){
    updateCurrentLocation(lat, lng);
    updateCompass(magxyz);
}

inline void D3Guide::updateCurrentLocation(double lat, double lng){
    mCurrentLat[1] = mCurrentLat[0];
    mCurrentLng[1] = mCurrentLng[0];
    
    mCurrentLat[0] = lat;
    mCurrentLng[0] = lng;
}

inline void D3Guide::updateCompass(int16_t magxyz[3]){
    mCompassRad = calcCompassRad(magxyz);
}

inline void D3Guide::setGoalLocation(double lat, double lng){
    mGoalLat = lat;
    mGoalLng = lng;
    mMagDeclination = toRadian(calcMagDeclination(lat, lng));
}

inline void D3Guide::setMagBias(int16_t xbias, int16_t ybias, int16_t zbias){
    mMagBiasxyz[0] = xbias;
    mMagBiasxyz[1] = ybias;
    mMagBiasxyz[2] = zbias;
}

inline void D3Guide::setMagBias(int16_t xyzbias[3]){
    for(int i=0; i<3; i++)
        mMagBiasxyz[i] = xyzbias[i];
}


inline void D3Guide::setLocationError(double error){
    mLocationError = error;
}


inline void D3Guide::setCompassAxis(compassAxis forward, compassAxis right){
    mForward = forward;
    mRight   = right;
}

// getTurnlRad : 機体固定座標系で前方からゴール方向までの角度を算出(CW正, -pi~pi)
//               compassRad:機体前方から北までの角度(CW正, -pi~pi)
//               gpsGoalRad:北からゴール方向までの角度(北0, 東pi/2, -pi~pi)
inline double D3Guide::getTurnRad(){
    double gpsGoalRad = calcGpsRad(mCurrentLat[0], mCurrentLng[0], mGoalLat, mGoalLng);
    return withinMPiToPi(mCompassRad + gpsGoalRad);
}

// getGoalDistace : 現在地からゴールまでの距離を算出(meters)
inline double D3Guide::getGoalDistance(){
    return distanceBetween(mCurrentLat[0], mCurrentLng[0], mGoalLat, mGoalLng);
}

inline bool D3Guide::RaspiIsUpdated(){
    return mRaspiIsUpdated;
}

inline double D3Guide::getCompassRad(){
    return mCompassRad; 
}

inline int16_t D3Guide::getRaspiX(){
    mRaspiIsUpdated = false;
    return mRaspiData[0];
}

inline int16_t D3Guide::getRaspiY(){
    mRaspiIsUpdated = false;    
    return mRaspiData[1];
}

inline double D3Guide::calcCompassRad(int16_t magxyz[3]){
    double rad = atan2(toFrameAxis(magxyz, mRight), toFrameAxis(magxyz, mForward)) - mMagDeclination;
    // return rad;
    return withinMPiToPi(rad);
}

//磁気偏角(度)の近似計算　緯度経度から求める（日本のみ）
//http://vldb.gsi.go.jp/sokuchi/geomag/menu_04/index.html
inline double D3Guide::calcMagDeclination(double lat, double lng){
    double dPhi = lat - 37.0;
    double dLam = lng - 138.0;
    return 7.0 + (57.201 + 18.750*dPhi - 6.761*dLam - 0.059*dPhi*dPhi - 0.014*dPhi*dLam - 0.579*dLam*dLam) / 60.0;
}

inline double D3Guide::toRadian(double deg){return deg * DEGREE_TO_RADIAN;}

#endif  //D3_GUIDE_H