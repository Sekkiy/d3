#ifndef D3_GUIDE_H
#define D3_GUIDE_H

#include <string>
#include <vector>
#include "mbed.h"

class D3Guide
{
public:
    D3Guide();

    enum CompassAxis{
        AXIS_X = 0,
        AXIS_Y,
        AXIS_Z,
        AXIS_mX,    //-X
        AXIS_mY,    //-Y
        AXIS_mZ     //-Z
    };

    enum CameraDirection{
        CABLE_UPWORD = 0,
        CABLE_DOWNWORD,
        CABLE_RIGHT,
        CABLE_LEFT
    };


    void updateCurrentLocation(double lat, double lng);
    void updateCompass(int16_t magxyz[3]);
    void updateInputs(double lat, double lng, int16_t magxyz[3]);
    void setGoalLocation(double lat, double lng);
    void setMagBias(int16_t xbias, int16_t ybias, int16_t zbias);
    void setMagBias(int16_t xyzbias[3]);
    void setLocationError(double error);
    void setCompassAxis(CompassAxis forward, CompassAxis right);
    void setCameraDirection(CameraDirection cableDirect);
    double getCompassRad();
    double getTurnRad();
    double getGoalDistance();
    void divideVelocity(double val[2], double velocity, double course);
    
    void encode(char c);
    int16_t getRaspiX();
    int16_t getRaspiY();
    float getRaspiLength();
    bool RaspiIsUpdated();
    bool RedIsInCamrera();

private:
    double mCurrentLat[2];
    double mCurrentLng[2];
    double mGoalLat;
    double mGoalLng;
    double mCompassRad;
    double mMagDeclination;
    double mLocationError;
    int16_t mMagBiasxyz[3];
    CompassAxis mForward;
    CompassAxis mRight;
    CameraDirection mCableDirect;
    
    std::string mStr;
    bool mRaspiIsUpdated;
    bool mRedIsinCamere;
    int16_t mRaspiData[2];

    constexpr static double DEGREE_TO_RADIAN = 0.017453292519943295769236907684886;
    constexpr static size_t MAX_RASPI_SENTENCE = 11 - 2;  //全出力数からスタートビットとストップビットを除いた数 

    double calcCompassRad(int16_t magxyz[3]);    
    double calcGpsRad(double lat1, double long1, double lat2, double long2);
    double calcMagDeclination(double lat, double lng);
    double distanceBetween(double lat1, double long1, double lat2, double long2);
    double withinMPiToPi(double rad);
    double toRadian(double deg);

    bool decode(std::string& str);
    
    int16_t fromCompassToFrame(int16_t magxyz[3], CompassAxis axis);
    void fromCameraToFrame(int16_t framexy[2], int16_t cameraxy[2], CameraDirection direction);
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


inline void D3Guide::setCompassAxis(CompassAxis forward, CompassAxis right){
    mForward = forward;
    mRight   = right;
}

inline void D3Guide::setCameraDirection(CameraDirection cableDirect){
    mCableDirect = cableDirect;
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

inline bool D3Guide::RedIsInCamrera(){
    return mRedIsinCamere;
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

inline float D3Guide::getRaspiLength(){
    return sqrt(mRaspiData[0]*mRaspiData[0] + mRaspiData[1]*mRaspiData[1]);
}


inline double D3Guide::calcCompassRad(int16_t magxyz[3]){
    double rad = atan2(fromCompassToFrame(magxyz, mRight), fromCompassToFrame(magxyz, mForward)) - mMagDeclination;
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