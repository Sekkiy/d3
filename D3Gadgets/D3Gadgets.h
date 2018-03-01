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
    void setStartDecelerationDist(float meters){mStartDecelerationDist = meters;}
    void setDecelerationRate(float rate){mDecelerationRate = rate;}
    void nowControlling(bool isControlling = true){mIsControlling = isControlling;}

    float getTurnDeg(){return mTurnDeg;}
    float getGoalDist(){return mGoalDist;}

    void updateBaroData(float hPa){mRcch.setThrottoleWithRatio(mHoveringTHLRatio + hover.calcTHLRatio(hPa));}
    void updateCompassData(int16_t magxyz[3]){guide.updateCompass(magxyz);}
    void updateGPSData(double latitude, double longtitude, double velocity){
        guide.updateCurrentLocation(latitude,longtitude);
        mVelocity = velocity;
    }

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
    float mStartDecelerationDist;
    float mDecelerationRate;
    bool mEnteringGoal;

    void setMoveParams();
    void moveByTime(MoveComand comand, float second);
    void guideLocation();
    void stopLateralMotion();
    void stopVerticalMotion();
    void stopMotion();
    void calcToGoalParams();
    uint16_t pitchByDeg(float deg);
    uint16_t yawByDeg(float deg);
    float pitchRateByGoal(float meters);
    // uint16_t pitchByDeg(float deg);
    // uint16_t yawByDeg(float deg);
    float fline(float val, float x0, float y0, float x1, float y1);
};

inline void D3Gadgets::calcToGoalParams(){
    mTurnDeg = static_cast<float>(guide.getTurnRad()) * 180.0/M_PI;
    mGoalDist = static_cast<float>(guide.getGoalDistance());
}


//点0~点1の2点間の直線
inline float D3Gadgets::fline(float val, float x0, float y0, float x1, float y1){
    return (y1-y0)/(x1-x0) * (val-x0) + y0;
}


#endif //D3GADGETS_H