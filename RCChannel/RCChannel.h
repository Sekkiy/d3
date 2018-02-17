#ifndef RCCHANNEL
#define RCCHANNEL

#include <vector>
#include <utility>
#include "mbed.h"

using namespace std;

class RCChannel
{
public:
    static const uint16_t CH_MIN = 900;
    static const uint16_t CH_MAX = 2100;
    static const uint16_t CH_NEUTRAL = 1500;

    enum ChName{
        AIL = 0,
        ELE,
        THR,
        RUD,
        AUX1,
        AUX2,
        AUX3,
        AUX4,
        AUX5,
        AUX6,
    };

    RCChannel(size_t chNum);

    bool setValue(uint8_t ch, uint16_t value);
    void setValue(const vector<uint16_t> &channels);
    void setAileron(uint16_t value);
    void setElevator(uint16_t value);
    void setThrottole(uint16_t value);
    void setRudder(uint16_t value);

    void setThrottoleWithRatio(float ratio);

    uint16_t value(uint8_t ch);
    uint16_t value(ChName ch);
    uint16_t aileron();
    uint16_t elevator();
    uint16_t throttole();
    uint16_t rudder();
    bool isReverse(uint8_t ch);

    void reverse(uint8_t ch, bool enable);
    void reset();

private:
    vector< pair<uint16_t, bool> > m_channels;
    const uint8_t m_chNum;

    uint16_t chRange;

    uint16_t withinRange(uint16_t value) const;
};

inline void RCChannel::reverse(uint8_t ch, bool enable){m_channels[ch].second = enable;}

inline void RCChannel::setAileron(uint16_t value)   {setValue(AIL, value);}
inline void RCChannel::setElevator(uint16_t value)  {setValue(ELE, value);}
inline void RCChannel::setThrottole(uint16_t value) {setValue(THR, value);}
inline void RCChannel::setRudder(uint16_t value)    {setValue(RUD, value);}

inline void RCChannel::setThrottoleWithRatio(float ratio){
    setThrottole(static_cast<uint16_t>(ratio * chRange + CH_MIN));
}

inline uint16_t RCChannel::value(uint8_t ch) {return m_channels[ch].first;}
inline uint16_t RCChannel::value(ChName ch)  {return value(static_cast<uint8_t>(ch));}
inline uint16_t RCChannel::aileron()         {return value(AIL);}
inline uint16_t RCChannel::elevator()        {return value(ELE);}
inline uint16_t RCChannel::throttole()       {return value(THR);}
inline uint16_t RCChannel::rudder()          {return value(RUD);}


inline bool RCChannel::isReverse(uint8_t ch){return m_channels[ch].second;}

#endif
