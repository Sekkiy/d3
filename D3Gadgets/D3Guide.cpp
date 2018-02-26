#include "D3Guide.h"
#include <math.h>


//divideVelocity : 機体の速度を前後と左右方向に分ける．
//                 val[2]:速度を格納する配列．val[0]:機体前後速度(前正), val[1]:機体左右速度(右正)
//                 checkPresiciton:精度チェック(default:true)
//                 設定した距離よりも移動量が小さければ精度が確証できないとしてfalseを返す
bool D3Guide::divideVelocity(double val[2], double velocity, bool checkPrecision){
    double gpsMovingDirectionRad = calcGpsRad(mCurrentLat[1], mCurrentLng[1], mCurrentLat[0], mCurrentLng[0]);
    double movingDirectionRad = withinMPiToPi(mCompassRad + gpsMovingDirectionRad);
    val[0] = velocity * cos(movingDirectionRad);
    val[1] = velocity * sin(movingDirectionRad);
    if(checkPrecision && distanceBetween(mCurrentLat[1], mCurrentLng[1], mCurrentLat[0], mCurrentLng[0]) < mLocationError)
        return false;
    else
        return true; 
}

//TinyGpsPlusのcourseToを改造
//位置1->位置2への向きを北0，東pi/2で出力 -pi~pi
double D3Guide::calcGpsRad(double lat1, double long1, double lat2, double long2){
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = toRadian(long2-long1);
  lat1 = toRadian(lat1);
  lat2 = toRadian(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  return a2;
}

int16_t D3Guide::toFrameAxis(int16_t magxyz[3], compassAxis axis){
    switch(axis){
        case AXIS_X:
            return magxyz[0]-mMagBiasxyz[0];
            break;
        case AXIS_Y:
            return magxyz[1]-mMagBiasxyz[1];
            break;
        case AXIS_Z:
            return magxyz[2]-mMagBiasxyz[2];
            break;
        case AXIS_mX:
            return -(magxyz[0]-mMagBiasxyz[0]);
            break;
        case AXIS_mY:
            return -(magxyz[1]-mMagBiasxyz[1]);
            break;
        case AXIS_mZ:
            return -(magxyz[2]-mMagBiasxyz[2]);
            break;
    }
}

//TinyGps++より抜粋.gpsから距離を算出(meter)
double D3Guide::distanceBetween(double lat1, double long1, double lat2, double long2){
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = toRadian(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = toRadian(lat1);
  lat2 = toRadian(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = delta*delta;
  delta += clat2 * clat2 * sdlong * sdlong;
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double D3Guide::withinMPiToPi(double rad){
    if(rad > M_PI)
        rad -= 2*M_PI;
    if(rad < -M_PI)
        rad += 2*M_PI;
    return rad;
}