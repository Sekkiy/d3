#include "mbed.h"
#include "PPMIn.h"

PpmIn::PpmIn(PinName pin, uint8_t channelNum): m_ppm(pin)
{
    m_channelNum = channelNum;
    m_channels.resize(channelNum);
    m_currentChannel = 0;
    m_state = false;
    m_timer.start();
    m_ppm.rise(this, &PpmIn::rise);
       
}

vector<uint16_t> PpmIn::getChannel() const{
    return m_channels;
}

void PpmIn::getChannels(vector<uint16_t>& channels){
    channels = m_channels;
}

uint16_t PpmIn::getChannel(uint8_t channel) const{
    return m_channels[channel];             
}

void PpmIn::rise()
{
    uint16_t time = m_timer.read_us();
    
    // we are in synchro zone
    if(time > 2500)
    {
       m_currentChannel = 0;
       // return values 
       m_state = true;
    }
    else
    {
        m_channels[m_currentChannel] = m_timer.read_us();
        m_channels[m_currentChannel] -= error(m_channels[m_currentChannel]);
        m_currentChannel += 1;     
    }
    
    m_timer.reset();
    
    //if (m_currentChannel > (CHANNELS + 2 - 1)); //+frame and - 1 indexing of m_channels list
}