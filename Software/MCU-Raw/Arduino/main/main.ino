/*
 *  
 */



// Includes
#include <avr/wdt.h>
#include <EEPROM.h>
#include "Configuration.h"
#include "InOut.h"
#include "Dimmer.h"

// Globals
// 1 - 1 output; 2 - 2 outputs
uint8_t HardwareVariant;                    // Auto detecting connected hardware
bool RollerShutter = false;                 // false - lighting; true - roller shutter
bool Monostable = false;
bool RememberStates = false;

//uint8_t DimmingLevelOff;
//uint8_t DimmingLevelOn;
uint8_t ValuesOff[3] = {R_VALUE_OFF, G_VALUE_OFF, B_VALUE_OFF};
uint8_t ValuesOn[3] = {R_VALUE_ON, G_VALUE_ON, B_VALUE_ON};

uint32_t RSTimer;
bool RSReset = false;

uint16_t EPPROM_Address[2] = {EEA_RELAY_1, EEA_RELAY_2};

// Constructors
InOut IO[NUMBER_OF_BUTTONS];

Dimmer D[NUMBER_OF_BUTTONS];

// Before
void before() {

  #ifdef ENABLE_WATCHDOG
    wdt_reset();
    MCUSR = 0;
    wdt_disable();
  #endif

}

// Setup
void setup()  {

  #ifdef ENABLE_WATCHDOG
    wdt_enable(WDTO_4S);
  #endif

  //float Vcc = ReadVcc();  // mV

  Serial.begin(115200);

  // Temporary code to substitute hardware variant detection
  #ifdef SINGLE_RELAY
    HardwareVariant = 1;
  #elif defined(DOUBLE_RELAY)
    HardwareVariant = 2;
  #endif

  // Hardware auto detection
  /*uint16_t ADCValue = analogRead(VERSION_DETECT_PIN);

  // Version A - 1 relay, 1 button
  if(ADCValue < 20) {
    Version = 1;  
  }
  // Version B - 2 relays, 2 buttons
  else if(ADCValue > 20 && ADCValue < 30)  {
    Version = 2;  
  }
  // Version DC - 2 open collector outputs
  else if(ADC > 30 && ADCValue < 40)  {
    Version = 3;
  }*/

  pinMode(DIP_SWITCH_1, INPUT_PULLUP);
  pinMode(DIP_SWITCH_2, INPUT_PULLUP);
  pinMode(DIP_SWITCH_3, INPUT_PULLUP);
  pinMode(DIP_SWITCH_4, INPUT_PULLUP);

  delay(100);

  // Reading dip switch 1
  if(!digitalRead(DIP_SWITCH_1)) {
    if(HardwareVariant == 2)  {
      RollerShutter = true;
    }
  }

  // Reading dip switch 2
  if(!digitalRead(DIP_SWITCH_2)) {
    Monostable = true;
  }

  // Reading dip switch 3
  if(!digitalRead(DIP_SWITCH_3)) {
    if(!Monostable && !RollerShutter) {
      RememberStates = true;
      uint8_t RecoveredState;
      for(int i=0; i<HardwareVariant; i++)  {
        EEPROM.get(EPPROM_Address[i], RecoveredState);
        if(RecoveredState < 2)  {
          IO[i].NewState = RecoveredState;
        }
      }      
    }
  }

  // Reading dip switch 4
  if(!digitalRead(DIP_SWITCH_4)) {
    // clear eeprom
    // clear load variant, eeprom states
    // for future use
  }

  // One button variant
  if(HardwareVariant == 0)  {
    // Initialize LEDs
    D[0].SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_4, LED_PIN_5, LED_PIN_6);

    // Show initialization
    RainbowLED(INIT_RAINBOW_DURATION, INIT_RAINBOW_RATE);

    // Initializing and calibrating button
    IO[0].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_3, INPUT_PIN_1, RELAY_PIN_1);

    // Restore saved state
    if(RememberStates)  {
      IO[0].SetRelay();
      AdjustLEDs(IO[0].NewState, 0);
    }
    else  {
      // Turn on LED
      AdjustLEDs(false, 0);
    }
  }
  // Two buttons variant
  else if(HardwareVariant == 1) {
    // Initialize LEDs
    D[0].SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_1, LED_PIN_2, LED_PIN_3);
    D[1].SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_7, LED_PIN_8, LED_PIN_9);

    // Show initialization
    RainbowLED(INIT_RAINBOW_DURATION, INIT_RAINBOW_RATE);
    
    // Initializing and calibrating buttons
    IO[0].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_1, INPUT_PIN_1, RELAY_PIN_1);
    IO[1].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_2, INPUT_PIN_2, RELAY_PIN_2);

    // Restore saved states
    if(RememberStates)  {
      for(int i=0; i<2; i++)  {
        IO[i].SetRelay();
        AdjustLEDs(IO[i].NewState, i);
      }
    }
    else  {
      // Turning on LEDs
      for(int i=0; i<2; i++)  {
        AdjustLEDs(false, i);
      }
    }
  }
}

// Builtin LEDs rainbow effect
void RainbowLED(uint16_t Duration, uint8_t Rate)	{
	
	//int RValue = 254;
	//int GValue = 127;
	//int BValue = 1;
  int RDirection = -1;
  int GDirection = -1;
  int BDirection = 1;
  int ColorValues[3] = {254, 127, 1};           // R, G, B
  uint32_t StartTime = millis();
	
  while(millis() < StartTime + Duration)	{

    if(HardwareVariant == 0)  {
      for(int i=0; i<NUMBER_OF_CHANNELS; i++) {
        D[0].NewValues[i] = ColorValues[i];
      }
      D[0].NewDimmingLevel = BRIGHTNESS_VALUE_ON;
      D[0].UpdateDimmer();
    }
    else if(HardwareVariant == 1) {
      for(int i=0; i<NUMBER_OF_CHANNELS; i++) {
        D[0].NewValues[i] = ColorValues[i];
        D[1].NewValues[i] = ColorValues[i];
      }
      for(int j=0; j<2; j++)  {
        D[j].NewDimmingLevel = BRIGHTNESS_VALUE_ON;
        D[j].UpdateDimmer();
      }
    }
	
    ColorValues[0] += RDirection;
    ColorValues[1] += GDirection;
    ColorValues[2] += BDirection;
	
    if(ColorValues[0] >= 255 || ColorValues[0] <= 0)	{
      RDirection = -RDirection;
    }
	
    if(ColorValues[1] >= 255 || ColorValues[1] <= 0)	{
      GDirection = -GDirection;
    }
	
    if(ColorValues[2] >= 255 || ColorValues[2] <= 0)	{
      BDirection = -BDirection;
    }
    delay(Rate);
  }
}

// Adjust LEDs
void AdjustLEDs(bool State, uint8_t Dimmer) {

  if(State != 1) {
    for(int i=0; i<NUMBER_OF_CHANNELS; i++) {
      D[Dimmer].NewValues[i] = ValuesOff[i];
    }
    D[Dimmer].NewDimmingLevel = BRIGHTNESS_VALUE_OFF;
  }
  else  {
    for(int i=0; i<NUMBER_OF_CHANNELS; i++) {
      D[Dimmer].NewValues[i] = ValuesOn[i];
    }
    D[Dimmer].NewDimmingLevel = BRIGHTNESS_VALUE_ON;
  }

  D[Dimmer].UpdateDimmer();
}

/*// ReadVcc
long ReadVcc() {
  
  long result;
  
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  
  delay(2);
  
  ADCSRA |= _BV(ADSC); // Convert
  
  while (bit_is_set(ADCSRA,ADSC));
  
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  result = result;
  
  return result;
}*/

// Loop
void loop() {

  // Check inputs & adjust outputs
  for(int i=0; i<HardwareVariant; i++)  {
    IO[i].ReadInput(TOUCH_THRESHOLD, /*LONGPRESS_DURATION,*/ DEBOUNCE_VALUE, Monostable);
    if(IO[i].NewState != IO[0].OldState)  {
      if(RollerShutter) {
        if(IO[0].OldState || IO[1].OldState)  {
          // Stop
          for(int j=0; j<HardwareVariant; j++)  {
            IO[j].NewState = 0;            
            IO[j].SetRelay();
            AdjustLEDs(false, j);
          }
        }
        else  {
          IO[i].SetRelay();
          AdjustLEDs(IO[i].NewState, i);
          if(IO[i].NewState)  {
            RSTimer = millis();
            RSReset = true;
          }
        }
      }
      else  {
        IO[i].SetRelay();
        AdjustLEDs(IO[i].NewState, i);
        // Saving state to eeprom
        //EEPROM.put(EPPROM_Address[i], IO[i].NewState);
      }
    }
  }

  // Roller shutter timer  
  if(RollerShutter == true) {
    if(millis() > RSTimer + RS_INTERVAL && RSReset)  {
      for(int i=0; i<2; i++)  {
        IO[i].NewState = 0;
        IO[i].SetRelay();
        AdjustLEDs(false, i);
      }
      RSReset = false;
    }
  }
  

}
