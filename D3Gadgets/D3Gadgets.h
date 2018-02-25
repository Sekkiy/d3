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
    void nowControlling(bool isControlling = true){mIsControlling = isControlling;}

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
    RCChannel mRcch;
    bool mIsControlling;
    float mTurnDeg;
    uint16_t mMaxIncYaw;
    uint16_t mMaxIncPitch;
    
    
    void setMoveParams();
    void moveByTime(MoveComand comand, float second);
    void guideLocation();
    uint16_t pitchByDeg(float deg);
    uint16_t yawByDeg(float deg);
    float fline(float val, float x0, float y0, float x1, float y1);
};

//2点間の直線
inline float D3Gadgets::fline(float val, float x0, float y0, float x1, float y1){
    return (y1-y0)/(x1-x0) * (val-x0) + y0;
}


#endif //D3GADGETS_H