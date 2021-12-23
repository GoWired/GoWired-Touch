#include "InOut.h"

// Constructor
InOut::InOut()  {
  
  _NewState = 0;
  _State = 0;
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

  ReadReference();
}

// Measure touch field reference value
void InOut::ReadReference()  {

  _TouchReference = ADCTouch.read(_SensorPin, 500);         // ADCTouch.read(pin, number of samples)
}

// Return _NewState
uint8_t InOut::ReadNewState() {

  return _NewState;
}

uint8_t InOut::ReadState()  {

  return _State;
}

// Set _NewState
void InOut::SetState(uint8_t NewState)  {

  _NewState = NewState;
}

// Read analog & digital inputs
int InOut::ReadInput(uint16_t Threshold, uint16_t LongpressDuration, uint8_t DebounceValue)  {

  int TouchValue;
  int TouchValueAggregated;
  bool Shortpress = false;
  bool Longpress = false;
  uint8_t Counter = 0;
  uint32_t StartTime = millis();

  do {
    // Read touch field with 100 samples
    TouchValue = ADCTouch.read(_SensorPin);
    TouchValue -= _TouchReference;
    TouchValueAggregated += TouchValue;
    Counter++;
    
    if(TouchValue > Threshold)  {
      Shortpress = true;
    }
    else  {
      if(SensorType == 1) {
        Shortpress = ReadDigital(DebounceValue);
      }
    }

    if(millis() - StartTime > LongpressDuration) {
      Longpress = true;
      break;
    }

    if(millis() < StartTime)  {
      StartTime = millis();
    }

  } while(Shortpress);

  if(Longpress) {
    _NewState = 2;
  }
  else if(Shortpress) {
    _NewState = !_State;
  }

  return (int)(TouchValueAggregated / Counter);
}

bool InOut::ReadDigital(uint8_t DebounceValue) {

  bool DigitalInput;
  uint32_t StartTime = millis();

  while(millis() - StartTime < DebounceValue) {
    if(digitalRead(_SensorPin2))  {
      DigitalInput = false;
      break;
    }
    else  {
      DigitalInput = true;
    }

    if(millis() < StartTime)  {
      StartTime = millis();
    }
  }

  return DigitalInput;
}

// Set Relay
void InOut::SetRelay() {

  if(_NewState == 1)  {
    digitalWrite(_RelayPin, _RelayON);
  }
  else  {
    digitalWrite(_RelayPin, _RelayOFF);
  }
  
  _State = _NewState;
}
