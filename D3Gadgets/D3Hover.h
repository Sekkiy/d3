#ifndef D3_HOVER_H
#define D3_HOVER_H

#include <math.h>
#include "mbed.h"
#include "RCChannel.h"

// #define _DEBUG_D3HOVER
#ifdef _DEBUG_D3HOVER
extern RawSerial pc;
#define DEBUG_PRINT(...) pc.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

class Counter
{
public:
    Counter():mCount(0){}
    
    bool count(bool condition, int num){
        if(condition){
            mCount++;
            if(mCount >= num){
                mCount = 0;
                return true;
            }
        }else
            mCount = 0;
        return false;
    }
private:
    int mCount;
};

class D3Hover
{
public:
    D3Hover(RCChannel* rcch);

    uint16_t hovering(float pressure);

    uint16_t calcTHL(float pressure);
    float calcTHLRatio(float hPa);

    void setTargetPressure(float hPa);
    void setPgain(float Pgain);
    void setDgain(float Dgain);
    void setMaxMinTHL(float max, float min);
    //void setHoveringTHL(uint16_t throttole);
    void setHoveringTHLRatio(float ratio);
    void setThresholdHeightRange(float range);

    float pressToHeight_m(float hPa);

    float dH,dV;//debug用

private:
    RCChannel* p_rcch;
    Counter counter;
    Timer t;

    float targetHeight;
    float Kp,Kd;
    float hoveringTHLRatio;
    float maxTHL, minTHL;
    float ddH_threshold;
    float velocity;

    float calcVelocity(float dH);
    
};

inline void D3Hover::setTargetPressure(float hPa){targetHeight = pressToHeight_m(hPa);}
inline void D3Hover::setPgain(float Pgain){Kp = Pgain;}
inline void D3Hover::setDgain(float Dgain){Kd = Dgain;}
inline void D3Hover::setHoveringTHLRatio(float ratio){hoveringTHLRatio = ratio;}
inline void D3Hover::setMaxMinTHL(float max, float min){
    maxTHL = max;
    minTHL = min;
}
inline void D3Hover::setThresholdHeightRange(float range){ddH_threshold = range;}


//気圧と高度の近似換算式 
//詳しくは http://www.tone-akadaya.com/entry/pressure-altitude
inline float D3Hover::pressToHeight_m(float hPa){
    return static_cast<float>((44.331514 - 11.880516 * pow(hPa, 0.1902632)) * 1000);
}



#endif //D3_HOVER_H