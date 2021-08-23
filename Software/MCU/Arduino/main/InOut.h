
#ifndef InOut_h
#define InOut_h

#include "Arduino.h"
#include "ADCTouch.h"

class InOut
{
  public:
    InOut();

    uint8_t SensorType;
    uint8_t NewState;
    uint8_t OldState;

    void SetValues(bool RelayOFF, bool RelayOn, uint8_t Type, uint8_t Pin1, uint8_t Pin2=0, uint8_t Pin3=0);
    void ReadInput(uint16_t Threshold, uint16_t LongpressDuration, uint8_t DebounceValue);
    //void DigitalInput();
    //void AnalogInput(uint16_t Threshold, bool Monostable);
    void SetRelay();

  private:
    uint16_t _TouchReference;
    uint8_t _RelayPin;
    uint8_t _SensorPin;
    uint8_t _SensorPin2;
    bool _RelayOFF;
    bool _RelayON;
};



#endif