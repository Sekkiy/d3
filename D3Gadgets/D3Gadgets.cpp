#include "D3Gadgets.h"
#include <math.h>

D3Gadgets::D3Gadgets(int chNum):
mRcch(chNum),
move(&mRcch),
hover(&mRcch)
{
    setMoveParams();
}

void D3Gadgets::process(RCChannel& rcch){
    if(mIsControlling){
        
    }
}

void D3Gadgets::setMoveParams(){
    move.setNeutralELE(DEFAULT::NTL_ELE);
    move.setForwardELE(DEFAULT::FWD_ELE);
    move.setBackELE(DEFAULT::BCK_ELE);   

    move.setNeutralAIL(DEFAULT::NTL_AIL);
    move.setRightAIL(DEFAULT::RGT_AIL);    
    move.setLeftAIL(DEFAULT::LFT_AIL);

    move.setNeutralRUD(DEFAULT::NTL_RUD);
    move.setRightTurnRUD(DEFAULT::RGT_TURN_RUD);
    move.setLeftTurnRUD(DEFAULT::LFT_TURN_RUD);
}

void D3Gadgets::moveByTime(MoveComand comand, float second){
    switch(comand){
        case MOVE_FORWARD:
            move.forward();
            mTimeout[0].attach(callback(&(this->move),&D3Move::neutralFB),second);
            break;
        case MOVE_BACK:
            move.back();
            mTimeout[0].attach(callback(&(this->move),&D3Move::neutralFB),second);
            break;
        case MOVE_RIGHT:
            move.right();
            mTimeout[1].attach(callback(&(this->move),&D3Move::neutralRL),second);
            break;
        case MOVE_LEFT:
            move.left();
            mTimeout[1].attach(callback(&(this->move),&D3Move::neutralRL),second);
            break;
        case MOVE_TURN_R:
            move.rightTurn();
            mTimeout[2].attach(callback(&(this->move),&D3Move::neutralTurn),second);
            break;
        case MOVE_TURN_L:
            move.leftTurn();
            mTimeout[2].attach(callback(&(this->move),&D3Move::neutralTurn),second);
            break;
        default:
            break;
    }
}

void D3Gadgets::guideLocation(){
    float turnDeg = static_cast<float>(guide.getTurnRad()) * 180.0/M_PI;
    float goalDist = static_cast<float>(guide.getGoalDistance());

    float absDeg = abs(turnDeg);
    move.forward(pitchByDeg(absDeg));
    if(turnDeg > 0)
        move.right(pitchByDeg(absDeg));
    else
        move.left(yawByDeg(absDeg));
}

uint16_t D3Gadgets::pitchByDeg(float absDeg){
    if(absDeg > mTurnDeg)
        return 0.0;
    else if(absDeg > 5.0)
        return static_cast<uint16_t>(fline(absDeg, mTurnDeg, 0.0, 5.0,  static_cast<float>(mMaxIncPitch)));
    else
        return mMaxIncPitch;
}

uint16_t D3Gadgets::yawByDeg(float absDeg){
    if(absDeg > mTurnDeg)
        return mMaxIncYaw;
    else
        return static_cast<uint16_t>(fline(absDeg, mTurnDeg, static_cast<float>(mMaxIncYaw), 0.0, 0.0));
}