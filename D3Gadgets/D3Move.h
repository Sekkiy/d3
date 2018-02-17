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
    void back()     {setRcchValue(RCChannel::ELE, backELE);}

    void right()    {setRcchValue(RCChannel::AIL, rightAIL);}
    void left()     {setRcchValue(RCChannel::AIL, leftAIL);}

    void rightTurn(){setRcchValue(RCChannel::RUD, rightTurnRUD);}
    void leftTurn() {setRcchValue(RCChannel::RUD, leftTurnRUD);}


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


#endif //D3_MOVE_H