#ifndef D3_GUIDE_H
#define D3_GUIDE_H

#include "mbed.h"

class D3Guide
{
public:
    D3Guide();

    void setCurrentLocation(double latitude, double longtitude);
    void setGoalLocation(double latitude, double longtitude);



private:
    double currentlat;
    double currentlng;
    double goalLat;
    double goalLng;

};


inline void D3Guide::setCurrentLocation(double latitude, double longtitude){
    currentlat = latitude;
    currentlng = longtitude;
}

inline void D3Guide::setGoalLocation(double latitude, double longtitude){
    goalLat = latitude;
    goalLng = longtitude;
}

#endif  //D3_GUIDE_H