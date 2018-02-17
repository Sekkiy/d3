#ifndef D3GADGETS_H
#define D3GADGETS_H

#include "RCChannel.h"

#include "D3Move.h"
#include "D3Hover.h"
#include "D3Config.h"


class D3Gadgets
{

public:
    D3Gadgets(int chNum = 8);
    
    D3Move move;
    D3Hover hover;
    
    RCChannel rcch;

private:
    void setMoveParams();
    
};

#endif //D3GADGETS_H