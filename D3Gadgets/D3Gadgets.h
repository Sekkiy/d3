#ifndef D3GADGETS_H
#define D3GADGETS_H

#include "RCChannel.h"

#include "D3Move.h"
#include "D3Hover.h"
#include "D3Guide.h"
#include "D3Config.h"


class D3Gadgets
{

public:
    D3Gadgets(int chNum = 8);
    
    D3Move move;
    D3Hover hover;
    D3Guide guide;
    
    void process(RCChannel& rcch);
    void setStartTurnDeg(float deg){mStartTurnDeg = deg;}
    void setMaxELEIncremental(uint16_t maxELEInc){mMaxELEInc = maxELEInc;}
    void setMaxRUDIncremental(uint16_t maxRUDInc){mMaxRUDInc = maxRUDInc;}
    void setGainVtoS(float gain){mGainVtoS = gain;}
    void setMaxStopSec(float second){mMaxStopSec = second;}
    void setHoveringTHLRatio(float ratio){mHoveringTHLRatio = ratio;}
    void setChangeSequenceDist(float meters){mChangeSequenceDist = meters;}
    void nowControlling(bool isControlling = true){mIsControlling = isControlling;}

    float getTurnDeg(){return mTurnDeg;}
    float getGoalDist(){return mGoalDist;}

    void updateBaroData(float hPa){mRcch.setThrottoleWithRatio(mHoveringTHLRatio + hover.calcTHLRatio(hPa));}
    void updateCompassData(int16_t magxyz[3]){guide.updateCompass(magxyz);}
    void updateGPSData(double latitude, double longtitude, double velocity){
        guide.updateCurrentLocation(latitude,longtitude);
        mVelocity = velocity;
    }

    uint16_t pitchByDeg(float deg);
    uint16_t yawByDeg(float deg);
    RCChannel mRcch;

private:
    enum MoveComand{
        MOVE_FORWARD,
        MOVE_BACK,
        MOVE_RIGHT,
        MOVE_LEFT,
        MOVE_TURN_R,
        MOVE_TURN_L
    };

    Timeout mTimeout[3];
    bool mIsControlling;
    float mStartTurnDeg;
    uint16_t mMaxELEInc;
    uint16_t mMaxRUDInc;
    double mVelocity;
    float mHoveringTHLRatio;
    float mGainVtoS;
    float mMaxStopSec;
    float mTurnDeg;
    float mGoalDist;
    float mChangeSequenceDist;

    void setMoveParams();
    void moveByTime(MoveComand comand, float second);
    void guideLocation();
    void stopLateralMotion();
    void stopVerticalMotion();
    void stopMotion();
    // uint16_t pitchByDeg(float deg);
    // uint16_t yawByDeg(float deg);
    float fline(float val, float x0, float y0, float x1, float y1);
};

//2点間の直線
inline float D3Gadgets::fline(float val, float x0, float y0, float x1, float y1){
    return (y1-y0)/(x1-x0) * (val-x0) + y0;
}


#endif //D3GADGETS_H