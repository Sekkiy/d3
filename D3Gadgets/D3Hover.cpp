#include "D3Hover.h"
#include <math.h>

D3Hover::D3Hover(RCChannel* rcch)
: p_rcch(rcch),
  mVelocity(0),
  mTargetHeight(0),
  mDH(0),
  mDdH_threshold(0.35)
{
    setPgain(0);
    setIgain(0);
    setDgain(0);
    mThresholdIntegral = false;
    t.start();
}

float D3Hover::calcTHLRatio(float hPa){
    mDH = mTargetHeight - pressToHeight_m(hPa); //[m]
    mDT = t.read();
    mIdH = calcIntegral(mDH,mDT);
    mDV = calcVelocity(mDH,mDT); //[m/s] 
    
    // float thl = hoveringTHLRatio + Kp * mDH - Kd * mDV;
    float thl =  mKp * mDH + mKi * mIdH + mKd * mDV;

    DEBUG_PRINT("%f\t%f\r\n",mDH, mDV);    
    
    if(thl > mMaxTHL) thl = mMaxTHL;
    if(thl < mMinTHL) thl = mMinTHL;

    return thl;
}

float D3Hover::calcVelocity(float dH, float dt){
    static float dH_current = dH;
    float ddH = dH - dH_current;
    if(counter.count(abs(ddH) > mDdH_threshold, 5)){
        dH_current = dH;
        mVelocity = ddH / dt;
        t.reset();        
    }
    return mVelocity;
}

float D3Hover::calcIntegral(float dH, float dt){
    mIntegral += dH * dt;
    if(mThresholdIntegral && abs(mIntegral) <= mIMax)
        mIntegral = mThresholdIntegral;
    return mIntegral;
}