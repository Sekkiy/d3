#include "mbed.h"
#include "PPMOut.h"

PpmOut::PpmOut(PinName pin, uint8_t channelNumber): m_ppm(pin) {
    if(channelNumber > MAX_CHANNELS){
        this->m_channelNumber = MAX_CHANNELS;
    }
    this->m_channelNumber = channelNumber;
    resetChannels();
    m_pulseOut = 1;
    m_ppm = m_pulseOut;
    m_currentDot = 0;
    m_timeout.attach_us(this, &PpmOut::attimeout, FRAME_LEN);
}

void PpmOut::setChannel(int channelNo, uint16_t value) {
    //__disable_irq();     // Disable Interrupts
    if(channelNo > m_channelNumber-1)   return;
    
    if(value > MAX_CHANNEL_VALUE){
        value = MAX_CHANNEL_VALUE;
    }
    m_dots[channelNo*2+1] = value - CHANNEL_SYNC;
    setFrameSync();
    //__enable_irq();     // Enable Interrupts
}

void PpmOut::setChannels(const vector<uint16_t> &vec) {
    //__disable_irq();     // Disable Interrupts
    if(vec.size() > m_channelNumber)  return;

    for(uint8_t ch=0; ch<m_channelNumber; ch++){
        if(vec[ch] > MAX_CHANNEL_VALUE){
            m_dots[ch*2+1] =  MAX_CHANNEL_VALUE - CHANNEL_SYNC;
        }else if(vec[ch] < MIN_CHANNEL_VALUE){
            m_dots[ch*2+1] = MIN_CHANNEL_VALUE - CHANNEL_SYNC;            
        }else{
            m_dots[ch*2+1] = vec[ch] - CHANNEL_SYNC;            
        }
    }
    setFrameSync();
    //__enable_irq();     // Enable Interrupts
}

inline void PpmOut::setFrameSync(){
    uint16_t sum_channels = 0;
    for(uint8_t channel = 0; channel < m_channelNumber; channel++) {
        sum_channels += m_dots[channel*2+1];
    }
    sum_channels += m_channelNumber*CHANNEL_SYNC;
    m_dots[m_channelNumber*2] = CHANNEL_SYNC;
    m_dots[m_channelNumber*2+1] = FRAME_LEN - CHANNEL_SYNC - sum_channels;
}

void PpmOut::attimeout() {
    m_pulseOut = !m_pulseOut;
    m_ppm = m_pulseOut;
    
    m_timeout.attach_us(this, &PpmOut::attimeout, m_dots[m_currentDot]);
    m_currentDot++;

    if(m_currentDot == m_channelNumber*2+2) { // 2 for FRAME_SYNC
        m_currentDot = 0;
    }
}

inline void PpmOut::resetChannels() {
    int8_t channel;
    
    m_currentDot = 0;
    memset(m_dots, 0x00, DOTS);
    for(channel = 0; channel < m_channelNumber; channel++) {
        m_dots[channel*2] = CHANNEL_SYNC;
        m_dots[channel*2+1] = 1500 - CHANNEL_SYNC;
    }
    setFrameSync();
}