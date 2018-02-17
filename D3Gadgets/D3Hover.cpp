#include "D3Hover.h"
#include "math.h"

D3Hover::D3Hover(RCChannel* rcch)
: p_rcch(rcch),
  velocity(0),
  ddH_threshold(0.35)
{
    setPgain(0);
    setDgain(0);
    t.start();
}

float D3Hover::calcTHLRatio(float hPa){
    dH = targetHeight - pressToHeight_m(hPa); //[m]
    dV = calcVelocity(dH); //[m/s] 

    // float thl = hoveringTHLRatio + Kp * dH - Kd * dV;
    float thl =  Kp * dH + Kd * dV;

    DEBUG_PRINT("%f\t%f\r\n",dH, dV);    
    
    if(thl > maxTHL) thl = maxTHL;
    if(thl < minTHL) thl = minTHL;

    return thl;
}

float D3Hover::calcVelocity(float dH){
    static float dH_current = dH;
    float ddH = dH - dH_current;
    if(counter.count(abs(ddH) > ddH_threshold, 5)){
        dH_current = dH;
        velocity = ddH / t.read();
        t.reset();
    }
    return velocity;
}
