#include "InOut.h"

// Constructor
InOut::InOut()  {
  
  NewState = 0;
  OldState = 0;
  _HighStateDetection = true;
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

void InOut::ReadInput(uint16_t Threshold, uint16_t LongpressDuration, uint8_t DebounceValue)  {

  bool ExternalButtonState = true;
  bool LowStateDetection = false;
  bool Condition = false;
  uint16_t Value;
  uint32_t StartTime = millis();
  
  do {
    Value = ADCTouch.read(_SensorPin) - _TouchReference;

    if(SensorType == 1) {
      ExternalButtonState = digitalRead(_SensorPin2);
    }

    if(millis() - StartTime > DebounceValue) {
      LowStateDetection = true;
    }

    if(millis() - StartTime > LongpressDuration) {
      Condition = true;
      break;
    }

    if(millis() < StartTime)  {
      StartTime = millis();
    }
  } while(!ExternalButtonState || Value > Threshold);

  if(!Condition && LowStateDetection)  {
    NewState = OldState == 1 ? 0 : 1;
  }
  else if(Condition) {
    NewState = 2;
  }
}

/*// Check digital input
void InOut::DigitalInput()  {

  if(digitalRead(_SensorPin2) != LOW) {
    _HighStateDetection = true;
    _LowStateDetection = false;
    _Condition = false;
  }
  else  {
    if(_HighStateDetection == true) {
      _LowStateDetection = true;
    }
  }
  if(_LowStateDetection == true)  {
    NewState = !OldState;
  }
}

// Check analog input
void InOut::AnalogInput(uint16_t Threshold, bool Monostable) {
  
  bool ButtonPressed = false;
  uint16_t Value = ADCTouch.read(_SensorPin);
  Value -= _TouchReference;

  if(Value > Threshold) {
    ButtonPressed = true;
  }

  if(Monostable != true)  {
    if(ButtonPressed == true) {
      NewState = !OldState;
    }
    else  {
      if(ButtonPressed == true) {
        NewState = 1;
      }
      else  {
        NewState = 0;
      }
    }
  }
}*/

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
