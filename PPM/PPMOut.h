#ifndef CH_PPM_OUT
#define CH_PPM_OUT

#include <vector>

using namespace std;

class PpmOut{

public:
    static const uint8_t MAX_CHANNELS = 8;
    static const uint16_t CHANNEL_SYNC = 300; // us
    static const uint16_t FRAME_SYNC = 5000; // us
    static const uint16_t FRAME_LEN = 20000; // us
    static const uint16_t MAX_CHANNEL_VALUE = 2180; // us
    static const uint16_t MIN_CHANNEL_VALUE = 820; 
    static const uint16_t DOTS = MAX_CHANNELS*2+2; // two m_dots per channel + FRAME_SYNC

    /* Will start the PPM output */
    PpmOut(PinName pin, uint8_t channelNumber);
    /* Values go from MIN_CHANNEL_VALUE to MAX_CHANNEL_VALUE */
    void setChannel(int channel_no, uint16_t value);
    void setChannels(const vector<uint16_t> &vec);
    

private:
    /* These are the time m_dots where the signal changes the value 
       from 0 to 1 and in reverse */
    uint16_t m_dots[DOTS];
    Timeout m_timeout;
    DigitalOut m_ppm;
    uint8_t m_currentDot;
    uint8_t m_channelNumber;
    uint16_t m_frameLength;
    uint16_t m_pulseOut;

    void attimeout();
    inline void resetChannels();
    inline void setFrameSync();
};

#endif // CH_PPM_OUT