#include "D3Gadgets.h"
#include <math.h>

D3Gadgets::D3Gadgets(int chNum):
mRcch(chNum),
move(&mRcch),
hover(&mRcch)
{
    setMoveParams();
    setStartTurnDeg(DEFAULT::START_TURN_DEGREE);
    setMaxELEIncremental(DEFAULT::ELE_INCLIMENTAL);
    setMaxRUDIncremental(DEFAULT::RUD_INCLIMENTAL);
    setGainVtoS(DEFAULT::GAIN_V_TO_S);
    setMaxStopSec(DEFAULT::MAX_STOP_SECOND);
    setHoveringTHLRatio(DEFAULT::THLOTTOLE_HOVERING);
    nowControlling(false);
}

void D3Gadgets::process(RCChannel& rcch){
    if(mIsControlling){
        guideLocation();
        if(mGoalDist < mChangeSequenceDist){

        }
        // for(int i=0; i<4; i++)
        //     rcch.setValue(i,mRcch.value(i));
        rcch.setValue(0,mRcch.value(0));
        rcch.setValue(1,mRcch.value(1));
        rcch.setValue(3,mRcch.value(3));
    }
}

void D3Gadgets::setMoveParams(){
    move.setNeutralAIL(DEFAULT::NTL_AIL);
    move.setRightAIL(DEFAULT::RGT_AIL);    
    move.setLeftAIL(DEFAULT::LFT_AIL);

    move.setNeutralELE(DEFAULT::NTL_ELE);
    move.setForwardELE(DEFAULT::FWD_ELE);
    move.setBackELE(DEFAULT::BCK_ELE);   

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
    mTurnDeg = static_cast<float>(guide.getTurnRad()) * 180.0/M_PI;
    mGoalDist = static_cast<float>(guide.getGoalDistance());

    float absDeg = abs(mTurnDeg);
    move.forward(pitchByDeg(absDeg));
    if(mTurnDeg > 0)
        move.rightTurn(yawByDeg(absDeg));
    else
        move.leftTurn(yawByDeg(absDeg));
}

void D3Gadgets::stopLateralMotion(){
    double v[2];
    guide.divideVelocity(v, mVelocity, false);
    float sec = mGainVtoS * v[1];
    if(sec > mMaxStopSec)
        sec = mMaxStopSec;
    if(v[1] > 0.0)
        moveByTime(MOVE_LEFT, sec);
    else
        moveByTime(MOVE_RIGHT, sec); 
}

void D3Gadgets::stopVerticalMotion(){
    double v[2];
    guide.divideVelocity(v, mVelocity, false);
    float sec = mGainVtoS * v[0];
    if(sec > mMaxStopSec)
        sec = mMaxStopSec;
    if(v[1] > 0.0)
        moveByTime(MOVE_BACK, sec);
    else
        moveByTime(MOVE_FORWARD, sec); 
}

void D3Gadgets::stopMotion(){
    double v[2];
    float sec[2]; 
    guide.divideVelocity(v, mVelocity, false);
    for(int i=0; i<2; i++){
        sec[i] = mGainVtoS * v[i];
        if(sec[i] < mMaxStopSec)
            sec[i] = mMaxStopSec;
    }

    if(v[0] > 0.0)
        moveByTime(MOVE_BACK, sec[0]);
    else
        moveByTime(MOVE_FORWARD, sec[0]); 
    if(v[1] > 0.0)
        moveByTime(MOVE_LEFT, sec[1]);
    else
        moveByTime(MOVE_RIGHT, sec[1]); 
}



uint16_t D3Gadgets::pitchByDeg(float absDeg){
    if(absDeg > mStartTurnDeg)
        return 0.0;
    else if(absDeg > 5.0)
        return static_cast<uint16_t>(fline(absDeg, mStartTurnDeg, 0.0, 5.0,  static_cast<float>(mMaxELEInc)));
    else
        return mMaxELEInc;
}

uint16_t D3Gadgets::yawByDeg(float absDeg){
    if(absDeg > mStartTurnDeg)
        return mMaxRUDInc;
    else
        return static_cast<uint16_t>(fline(absDeg, mStartTurnDeg, static_cast<float>(mMaxRUDInc), 0.0, 0.0));
}