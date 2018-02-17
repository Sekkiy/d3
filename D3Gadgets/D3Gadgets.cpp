#include "D3Gadgets.h"


D3Gadgets::D3Gadgets(int chNum):
rcch(chNum),
move(&rcch),
hover(&rcch)
{
    setMoveParams();
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

