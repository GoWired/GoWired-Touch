#include "InOut.h"

// Constructor
InOut::InOut()  {
  
  NewState = 0;
  OldState = 0;
}

// Set Values - initialization
void InOut::SetValues(bool RelayOFF, bool RelayON, uint8_t Type, uint8_t Pin1, uint8_t Pin2, uint8_t Pin3) {

  _RelayOFF = RelayOFF;
  _RelayON = RelayON;

  SensorType = Type;
  switch(SensorType)  {
    // Touch Field (Dimmers)
    case 0:
      _SensorPin = Pin1;
      break;
    // Touch Field + External button + Relay (2Relay)
    case 1:
      _SensorPin = Pin1;
      _SensorPin2 = Pin2;
      _RelayPin = Pin3;
      pinMode(_SensorPin2, INPUT_PULLUP);
      pinMode(_RelayPin, OUTPUT);
      digitalWrite(_RelayPin, _RelayOFF);
      break;
    // Default case
    default:
      break;
  }

  // Measure reference value
  _TouchReference = ADCTouch.read(_SensorPin, 500);
}

void InOut::ReadInput(uint16_t Threshold, uint8_t DebounceValue, bool Monostable)  {

  bool ExternalButtonState = true;
  bool LowStateDetection = false;
  int Value;
  uint32_t StartTime = millis();
  
  do {
    Value = ADCTouch.read(_SensorPin, 20);
    Value -= _TouchReference;

    if(SensorType == 1) {
      ExternalButtonState = digitalRead(_SensorPin2);
    }

    if(millis() - StartTime > DebounceValue) {
      LowStateDetection = true;
    }

    if(Monostable)  {
      if(LowStateDetection && !NewState) {
        NewState = 1;
        SetRelay();
      }
    }

    if(millis() < StartTime)  {
      StartTime = millis();
    }
  } while(!ExternalButtonState || Value > Threshold);

  if(!Monostable) {
    if(LowStateDetection) {
      NewState = OldState == 1 ? 0 : 1;
    }
  }
  else  {
    if(NewState)  {
      NewState = 0;
      SetRelay();
    }
  }
}

// Set Relay
void InOut::SetRelay() {

  if(NewState == 1)  {
    digitalWrite(_RelayPin, _RelayON);
  }
  else  {
    digitalWrite(_RelayPin, _RelayOFF);
  }
  
  OldState = NewState;
}
