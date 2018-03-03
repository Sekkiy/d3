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
    setStartDecelerationDist(DEFAULT::START_DECELERATION_DISTANCE);
    setDecelerationRate(DEFAULT::DECELERATION_RATE);
    mEnteringGoal = false;
    mGoalDist = 100;
    mCamTimer.start();
    nowControlling(false);
}

void D3Gadgets::process(RCChannel& rcch){
    calcToGoalParams();
    // checkSequence();

    guideLocationWithGPS();
    

    // switch(mSequence){
    //     case WAIT_FALL:
    //         break;
    //     case FALLING:
    //         break;
    //     case GUIDE_GPS:
    //         guideLocationWithGPS();
    //         break;
    //     case GUIDE_GPS_CLOSE:
    //         guideCloseLocationWithGPS();
    //         break;
    //     case GUIDE_CAMERA:
    //         guideLocationWithCamera();
    // }

    // if(mGoalDist < mChangeSequenceDist){
    //     // moveByTime(MOVE_BACK, 0.4);
    //     if(!mEnteringGoal){
    //         mEnteringGoal = true;
    //     }
    //     guideLocationWithCamera();
    // }else{
    //     guideLocationWithGPS();            
    //     if(mEnteringGoal)
    //         mEnteringGoal = false;
    // }
    if(mIsControlling){
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

void D3Gadgets::recoverFromFalling(){
    
}

void D3Gadgets::guideLocationWithGPS(){
    float absDeg = abs(mTurnDeg);

    guide.divideVelocity(mV, mVelocity, mGPSCourse);
    
    dV_FB = TargetForwardSpeed(mGoalDist) - mV[0];
    float dV_RL = 0.0 - mV[1];

    pitchVal = dV_FB * pitchByDegRate_forV(absDeg) * GAIN_PITCH;
    if(pitchVal > MAX_PITCH)
        pitchVal = MAX_PITCH;
    else if(pitchVal < -MAX_PITCH)
        pitchVal = -MAX_PITCH;


    if(pitchVal > 0.0)
        move.forward(static_cast<uint16_t>(pitchVal));
    else
        move.back(static_cast<uint16_t>(-pitchVal));

    move.neutralRL();
    // float rollVal = dV_RL * GAIN_ROLL;
    // if(rollVal > MAX_ROLL)
    //     rollVal = MAX_ROLL;
    // else if(rollVal < -MAX_ROLL)
    //     rollVal = -MAX_ROLL;

    // if(rollVal > 0.0)
    //     move.right(static_cast<uint16_t>(rollVal));
    // else
    //     move.left(static_cast<uint16_t>(-rollVal));

    // move.neutralRL();
    // move.forward(static_cast<uint16_t>(pitchRateByGoal(mGoalDist) * pitchByDeg(absDeg)
    //                                     +velocityControll(TARGET_V1,v[0])));
    
    if(mTurnDeg > 0)
        move.rightTurn(yawByDeg(absDeg));
    else
        move.leftTurn(yawByDeg(absDeg));
}

void D3Gadgets::guideCloseLocationWithGPS(){
    float turnRad = mTurnDeg * M_PI / 180.0;
    float fowardDist = mGoalDist * cos(turnRad);
    float rightDist = mGoalDist * sin(turnRad);

//
    double v[2];
    double TARGET_V2 = 1.0;
    guide.divideVelocity(v, mVelocity, mGPSCourse);
//

    move.neutralTurn();
    if(fowardDist > 0)
        move.forward(pitchByDist(abs(fowardDist)) /*+ velocityControll(TARGET_V2,v[0]) */);
    else
        move.back(pitchByDist(abs(fowardDist)) /*+ velocityControll(-TARGET_V2,v[0])*/);
    if(rightDist > 0)
        move.right(rollByDist(abs(rightDist)) /*+ velocityControll(TARGET_V2,v[1])*/);
    else
        move.left(rollByDist(abs(rightDist)) /*- velocityControll(TARGET_V2,v[1])*/);
}

void D3Gadgets::guideLocationWithCamera(){
    //高度補正どうしよ
    int16_t x = guide.getRaspiX(); //右正
    int16_t y = guide.getRaspiY(); //前正
    bool inRange = false;

    move.neutralTurn();
    if(sqrt(x*x + y*y) < mFinishLength){
        inRange  = true;
        move.neutral();
    }else{
        if(x > 0)
            move.right(mSlowAILInc);
        else
            move.left(mSlowAILInc);
        if(y > 0)
            move.forward(mSlowELEInc);
        else
            move.back(mSlowELEInc);
    }
    if(mCameraCounter.count(inRange, 5)){
        mSequence = GOAL;
    }
}

void D3Gadgets::stopLateralMotion(){
    double v[2];
    guide.divideVelocity(v, mVelocity, mGPSCourse);
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
    guide.divideVelocity(v, mVelocity, mGPSCourse);
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
    guide.divideVelocity(v, mVelocity, mGPSCourse);
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
        return 0;
    else if(absDeg > 5.0)
        return static_cast<uint16_t>(fline(absDeg, mStartTurnDeg, 0.0, 5.0,  static_cast<float>(mMaxELEInc)));
    else
        return mMaxELEInc;
}

float D3Gadgets::pitchByDegRate_forV(float absDeg){
    if(absDeg > mStartTurnDeg)
        return 0.0;
    else if(absDeg > 5.0)
        return fline(absDeg, mStartTurnDeg, 0.0, 5.0, 1.0);
    else
        return 1.0;
}

uint16_t D3Gadgets::yawByDeg(float absDeg){
    if(absDeg > mStartTurnDeg)
        return mMaxRUDInc;
    else
        return static_cast<uint16_t>(fline(absDeg, mStartTurnDeg, static_cast<float>(mMaxRUDInc), 0.0, 0.0));
}

float D3Gadgets::pitchRateByGoal(float meters){
    if(meters < mStartDecelerationDist)
        return fline(meters, mChangeSequenceDist, mDecelerationRate, mStartDecelerationDist, 1.0);
    else
        return 1.0;    
}


uint16_t D3Gadgets::pitchByDist(float meters){
    // if(meters < mChangeSequenceDist)
    //     return static_cast<uint16_t>(fline(meters, 0.0, 0.0, mChangeSequenceDist,mSlowELEInc));
    // else
        return mSlowELEInc;
}

uint16_t D3Gadgets::rollByDist(float meters){
    // if(meters < mChangeSequenceDist)
    //     return static_cast<uint16_t>(fline(meters, 0.0, 0.0, mChangeSequenceDist,mSlowAILInc));
    // else
        return mSlowAILInc;
}

void D3Gadgets::checkSequence(){
    if((mSequence == WAIT_FALL) && fall.checkFalling(mAccelxyz,mHeight))
        mSequence = FALLING;
    
    if(mGoalDist < mChangeSequenceDist){
        // if(guide.RedIsInCamera())
        //     mSequence = GUIDE_CAMERA;
        // else
            mSequence = GUIDE_GPS_CLOSE;
    }else
        mSequence = GUIDE_GPS;
}

float D3Gadgets::velocityControll(double targetSpeed, double nowSpeed){    
    float value =GAIN_V * (targetSpeed - nowSpeed);
    if(value > MAXINC_VC)
        value = MAXINC_VC;
    if(value <MININC_VC)
        value = MININC_VC;
    return value;
}

float D3Gadgets::TargetForwardSpeed(float goalDist){
    if(goalDist > mStartDecelerationDist)
        return TARGET_V1;
    else
        return fline(goalDist, mStartDecelerationDist, TARGET_V1, 0.0, 0.0);
}