#include "RCChannel.h"

RCChannel::RCChannel(size_t chNum) : m_chNum(chNum){
    chRange = CH_MAX - CH_MIN;
    m_channels.resize(m_chNum);
    reset();
}

bool RCChannel::setValue(uint8_t ch, uint16_t value){
    if(ch >= m_chNum)   return false;
    m_channels[ch].first = withinRange(value);
    return true;
}

void RCChannel::setValue(const vector<uint16_t> &channels){
    for(uint8_t i=0; i<m_chNum; i++){
        m_channels[i].first = withinRange(channels[i]);
    }
}

void RCChannel::reset(){
    for(uint8_t i=0; i<m_chNum; i++){
        if(i == 2){
            m_channels[i] = make_pair(CH_MIN, false);
        }else{
            m_channels[i] = make_pair(CH_NEUTRAL, false);
        }
    }
}

uint16_t RCChannel::withinRange(uint16_t value) const{
    if(value < CH_MIN) return CH_MIN;
    if(value > CH_MAX) return CH_MAX;
    return value;
}
