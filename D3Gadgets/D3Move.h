#ifndef D3_MOVE_H
#define D3_MOVE_H

#include "mbed.h"
#include "RCChannel.h"

class D3Move
{
public:
    D3Move(RCChannel* rcch): p_rcch(rcch){}
    
    void neutral()  {setRcchValue(RCChannel::ELE, neutralELE);
                     setRcchValue(RCChannel::AIL, neutralAIL);
                     setRcchValue(RCChannel::RUD, neutralRUD);}

    void forward()  {setRcchValue(RCChannel::ELE, forwardELE);}
    void forward(uint16_t incremental)  {setRcchValue(RCChannel::ELE, neutralELE + incremental);}
    void back()     {setRcchValue(RCChannel::ELE, backELE);}
    void back(uint16_t incremental)     {setRcchValue(RCChannel::ELE, neutralELE - incremental);}
    void stopForward(float mps);


    void right()    {setRcchValue(RCChannel::AIL, rightAIL);}
    void right(uint16_t incremental)    {setRcchValue(RCChannel::AIL, neutralAIL + incremental);}
    void left()     {setRcchValue(RCChannel::AIL, leftAIL);}
    void left(uint16_t incremental)     {setRcchValue(RCChannel::AIL, neutralAIL - incremental);}

    void rightTurn(){setRcchValue(RCChannel::RUD, rightTurnRUD);}
    void rightTurn(uint16_t incremental){setRcchValue(RCChannel::RUD, neutralRUD + incremental);}
    void leftTurn() {setRcchValue(RCChannel::RUD, leftTurnRUD);}
    void leftTurn(uint16_t incremental) {setRcchValue(RCChannel::RUD, neutralRUD - incremental);}


    void setNeutralELE(uint16_t value)  {neutralELE = value;}
    void setForwardELE(uint16_t value)  {forwardELE = value;}
    void setBackELE(uint16_t value)     {backELE    = value;}

    void setNeutralAIL(uint16_t value)  {neutralAIL = value;}
    void setRightAIL(uint16_t value)    {rightAIL   = value;}
    void setLeftAIL(uint16_t value)     {leftAIL    = value;}

    void setNeutralRUD(uint16_t value)  {neutralRUD     = value;}
    void setRightTurnRUD(uint16_t value){rightTurnRUD   = value;}
    void setLeftTurnRUD(uint16_t value) {leftTurnRUD    = value;}

private:
    RCChannel* p_rcch;

    uint16_t neutralELE;
    uint16_t forwardELE;
    uint16_t backELE;

    uint16_t neutralAIL;
    uint16_t rightAIL;
    uint16_t leftAIL;

    uint16_t neutralRUD;
    uint16_t rightTurnRUD;
    uint16_t leftTurnRUD;

    void setRcchValue(RCChannel::ChName ch, uint16_t value)   {p_rcch->setValue(ch, value);}
};

// void D3Move::stopForward(float mps){

// }



#endif //D3_MOVE_H