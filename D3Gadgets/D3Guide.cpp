#include "D3Guide.h"
#include <math.h>
#include <stdlib.h>
#include <sstream>

D3Guide::D3Guide():
mCableDirect(CABLE_DOWNWORD),
mRedIsinCamere(false)
{
    mStr.reserve(MAX_RASPI_SENTENCE);
}


//divideVelocity : 機体の速度を前後と左右方向に分ける．
//                 val[2]:速度を格納する配列．val[0]:機体前後速度(前正), val[1]:機体左右速度(右正)
void D3Guide::divideVelocity(double val[2], double velocity, double course){
    double courseRad = withinMPiToPi(toRadian(course));
    double movingDirectionRad = withinMPiToPi(mCompassRad + courseRad);
    val[0] = velocity * cos(movingDirectionRad);
    val[1] = velocity * sin(movingDirectionRad);
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

int16_t D3Guide::fromCompassToFrame(int16_t magxyz[3], CompassAxis axis){
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

//カメラ座標->機体座標
//カメラはケーブル下向きで正.xは画面右正,yは画面上正で中心が0　100分率で送ってくる
void D3Guide::fromCameraToFrame(int16_t framexy[2], int16_t cameraxy[2], CameraDirection direction){
    switch(direction){
        case CABLE_UPWORD:
            framexy[0] = -cameraxy[0];
            framexy[1] = -cameraxy[1];
            break;
        case CABLE_DOWNWORD:
            framexy[0] = cameraxy[0];
            framexy[1] = cameraxy[1];
            break;
        case CABLE_RIGHT:
            framexy[0] = -cameraxy[1];
            framexy[1] = cameraxy[0];
            break;
        case CABLE_LEFT:
            framexy[0] = cameraxy[1];
            framexy[1] = -cameraxy[0];
            break;
        default:
            framexy[0] = cameraxy[0];
            framexy[1] = cameraxy[1];
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


void D3Guide::encode(char c){
    switch(c){
        case 'S':
            mStr.clear();
            return;
        case 'F':
            decode(mStr);            
            return;
        default:
            mStr.push_back(c);
    }
}

bool D3Guide::decode(std::string& str){
    int16_t cameraxy[2];
    if(str.size() != MAX_RASPI_SENTENCE)
        return false;
    std::vector<std::string> strVec;
    strVec = split(str,',');
    for(int i=0; i<2; i++){
        cameraxy[i] = std::stoi(strVec[i]);;
    }
    if(cameraxy[0] == 999 && cameraxy[1] == 999){
        mRedIsinCamere = false;
        return true;
    }
    fromCameraToFrame(mRaspiData, cameraxy, mCableDirect);
    mRedIsinCamere = true;    
    mRaspiIsUpdated = true;
    return true;
}


//http://faithandbrave.hateblo.jp/entry/2014/05/01/171631 より抜粋
static std::vector<std::string> split(const std::string& input, char delimiter){
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}