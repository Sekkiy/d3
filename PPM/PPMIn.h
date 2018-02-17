#ifndef CH_PPM_IN
#define CH_PPM_IN

#include <vector>

using namespace std;

class PpmIn
{

public:
    static const uint8_t CHANNELS = 8;

    PpmIn(PinName pin, uint8_t channelNum = 8);
    void getChannels(vector<uint16_t>& channels);
    uint16_t getChannel(uint8_t channel) const;
    vector<uint16_t> getChannel() const;

private:
    InterruptIn m_ppm;
    Timer m_timer;
    vector<uint16_t> m_channels;
    uint8_t m_channelNum;
    uint16_t m_period;
    uint8_t m_currentChannel;
    bool m_state;

    void rise();
    uint16_t error(uint16_t value);
};


inline uint16_t PpmIn::error(uint16_t value){
    return (uint16_t)(0.0105 * (float)value - 1.015);
}


#endif