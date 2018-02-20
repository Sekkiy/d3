#ifndef D3_GUIDE_H
#define D3_GUIDE_H

#include "mbed.h"

class D3Guide
{
public:
    D3Guide();

    void setCurrentLocation(double latitude, double longtitude);
    void setGoalLocation(double latitude, double longtitude);
    void toGoal();


private:
    double currentlat[2];
    double currentlng[2];
    double goalLat;
    double goalLng;

    enum Condition{
        Stop,

    } condition;


    double getTurnRad();
};


inline void D3Guide::setCurrentLocation(double latitude, double longtitude){
    currentlat[1] = currentlat[0];
    currentlng[1] = currentlng[0];
    
    currentlat[0] = latitude;
    currentlng[0] = longtitude;
}

inline void D3Guide::setGoalLocation(double latitude, double longtitude){
    goalLat = latitude;
    goalLng = longtitude;
}

#endif  //D3_GUIDE_H