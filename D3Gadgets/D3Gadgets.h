#ifndef D3GADGETS_H
#define D3GADGETS_H

#include "RCChannel.h"

#include "D3Move.h"
#include "D3Hover.h"
#include "D3Guide.h"
#include "D3Fall.h"
#include "D3Config.h"
#include "Counter.h"

class D3Gadgets
{
public:
    constexpr static float GAIN_V = 2.0;
    constexpr static float MAXINC_VC = 0;
    constexpr static float MININC_VC = -50;
    constexpr static double TARGET_V2 = 0.5;

    constexpr static float GAIN_PITCH = 80;
    constexpr static float GAIN_ROLL = 54;
    constexpr static float MAX_PITCH = 90;
    constexpr static float MAX_ROLL = 90;
    constexpr static double TARGET_V1 = 1.5;


    enum Sequence{
        WAIT_FALL,
        FALLING,
        GUIDE_GPS,
        GUIDE_GPS_CLOSE,
        GUIDE_CAMERA,
        GOAL,
        FINISH_CONTROLL
    } mSequence;

    D3Gadgets(int chNum = 8);
    
    D3Move move;
    D3Hover hover;
    D3Guide guide;
    D3Fall fall;

    void process(RCChannel& rcch);
    void setStartTurnDeg(float deg){mStartTurnDeg = deg;}
    void setMaxELEIncremental(uint16_t maxELEInc){mMaxELEInc = maxELEInc;}
    void setMaxRUDIncremental(uint16_t maxRUDInc){mMaxRUDInc = maxRUDInc;}
    void setSlowAILIncremental(uint16_t slowAILInc){mSlowAILInc = slowAILInc;}
    void setSlowELEIncremental(uint16_t slowELEInc){mSlowELEInc = slowELEInc;}
    void setGainVtoS(float gain){mGainVtoS = gain;}
    void setMaxStopSec(float second){mMaxStopSec = second;}
    void setHoveringTHLRatio(float ratio){mHoveringTHLRatio = ratio;}
    void setChangeSequenceDist(float meters){mChangeSequenceDist = meters;}
    void setStartDecelerationDist(float meters){mStartDecelerationDist = meters;}
    void setNoSteerDist(float meters){}
    void setDecelerationRate(float rate){mDecelerationRate = rate;}
    void setFinishLength(float length){mFinishLength = length;} //  0~100
    void nowControlling(bool isControlling = true){mIsControlling = isControlling;}

    float getTurnDeg(){return mTurnDeg;}
    float getGoalDist(){return mGoalDist;}
    enum Sequence getSequence(){return mSequence;}
    bool isControlling(){return mIsControlling;}
    void updateBaroData(float hPa){
        // mRcch.setThrottoleWithRatio(mHoveringTHLRatio + hover.calcTHLRatio(hPa));
        mHeight = hover.pressToHeight_m(hPa);
    }
    void updateCompassData(int16_t magxyz[3]){guide.updateCompass(magxyz);}
    void updateGPSData(double latitude, double longtitude, double velocity, double course){
        guide.updateCurrentLocation(latitude,longtitude);
        mVelocity = velocity;
        mGPSCourse = course;
    }
    void updateAccelData(float accelxyz[3]){
        for(int i=0; i<3; i++)
            mAccelxyz[i] = accelxyz[i];
    }
    void encodePiSerial(char c){guide.encode(c);}

    //デバッグ用
    double mV[2];
    RCChannel mRcch;
    float dV_FB;
    float pitchVal;

private:
    enum MoveComand{
        MOVE_FORWARD,
        MOVE_BACK,
        MOVE_RIGHT,
        MOVE_LEFT,
        MOVE_TURN_R,
        MOVE_TURN_L
    };

    Counter mCameraCounter;
    Counter mSeqCounter;

    Timer mCamTimer;
    Timeout mTimeout[3];
    bool mIsControlling;
    float mStartTurnDeg;
    uint16_t mMaxELEInc;
    uint16_t mMaxRUDInc;
    uint16_t mSlowAILInc;
    uint16_t mSlowELEInc;
    double mVelocity;
    double mGPSCourse;
    float mHoveringTHLRatio;
    float mGainVtoS;
    float mMaxStopSec;
    float mTurnDeg;
    float mGoalDist;
    float mNoSteerDist;
    float mChangeSequenceDist;
    float mStartDecelerationDist;
    float mDecelerationRate;
    bool mEnteringGoal;
    float mFinishLength;

    //D3Fall関連
    float mHeight;
    float mAccelxyz[3];


    void setMoveParams();
    void moveByTime(MoveComand comand, float second);
    void recoverFromFalling();
    void guideLocationWithGPS();
    void guideCloseLocationWithGPS();
    void guideLocationWithCamera();
    void stopLateralMotion();
    void stopVerticalMotion();
    void stopMotion();
    void calcToGoalParams();
    uint16_t pitchByDeg(float absDeg);
    uint16_t yawByDeg(float absDeg);
    float pitchRateByGoal(float meters);
    uint16_t pitchByDist(float meters);
    uint16_t rollByDist(float meters);
    void checkSequence();
    float velocityControll(double targetSpeed, double nowSpeed);
    float TargetForwardSpeed(float goalDist);
    float pitchByDegRate_forV(float absDeg);
    
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