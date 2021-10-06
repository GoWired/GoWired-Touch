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

uint32_t LastCheck = 0;

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

  //float Vcc = ReadVcc();  // mV

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
  if(HardwareVariant == 1)  {
    // Initialize LEDs
    D[0].SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_7, LED_PIN_8, LED_PIN_9);

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
  else if(HardwareVariant == 2) {
    // Initialize LEDs
    D[0].SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_1, LED_PIN_2, LED_PIN_3);
    D[1].SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_4, LED_PIN_5, LED_PIN_6);

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

  #ifdef ENABLE_WATCHDOG
    wdt_enable(WDTO_4S);
  #endif
}

// Builtin LEDs rainbow effect
void RainbowLED(uint16_t Duration, uint8_t Rate)	{
	
  int RValue = 254;
  int GValue = 127;
  int BValue = 1;
  int RDirection = -1;
  int GDirection = -1;
  int BDirection = 1;
  uint32_t StartTime = millis();
	
  while(millis() < StartTime + Duration)	{

    for(int i=1; i<=HardwareVariant; i++)  {
      D[i-1].UpdateLEDs(BRIGHTNESS_VALUE_ON, RValue, GValue, BValue);
    }
	
    RValue += RDirection;
    GValue += GDirection;
    BValue += BDirection;
	
    if(RValue >= 255 || RValue <= 0)	{
      RDirection = -RDirection;
    }
	
    if(GValue >= 255 || GValue <= 0)	{
      GDirection = -GDirection;
    }
	
    if(BValue >= 255 || BValue <= 0)	{
      BDirection = -BDirection;
    }
    delay(Rate);
  }
}

// Adjust LEDs
void AdjustLEDs(bool State, uint8_t Dimmer) {

  if(State != 1) {
    D[Dimmer].UpdateLEDs(BRIGHTNESS_VALUE_OFF, R_VALUE_OFF, G_VALUE_OFF, B_VALUE_OFF);
  }
  else  {
    D[Dimmer].UpdateLEDs(BRIGHTNESS_VALUE_ON, R_VALUE_ON, G_VALUE_ON, B_VALUE_ON);
  }
}

// Check Inputs and adjust outputs
void UpdateIO() {

  for(int i=0; i<HardwareVariant; i++)  {
    IO[i].ReadInput(TOUCH_THRESHOLD, /*LONGPRESS_DURATION,*/ DEBOUNCE_VALUE, Monostable);
    if(IO[i].NewState != IO[i].OldState)  {
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
        if(!Monostable) {
          IO[i].SetRelay();
          AdjustLEDs(IO[i].NewState, i);
        }
        // Saving state to eeprom
        //EEPROM.put(EPPROM_Address[i], IO[i].NewState);
      }
    }
  }
}

// Loop
void loop() {

  #ifdef ENABLE_WATCHDOG
    wdt_reset();
  #endif

  // Check inputs & adjust outputs
  if(millis() > LastCheck + LOOP_TIME)  {
    LastCheck = millis();
    UpdateIO();
  }
  else if(millis() < LastCheck) {
    LastCheck = millis();
  }

  // Roller shutter timer  
  if(RollerShutter == true) {
    if((millis() > RSTimer + RS_INTERVAL) && RSReset)  {
      for(int i=0; i<2; i++)  {
        IO[i].NewState = 0;
        IO[i].SetRelay();
        AdjustLEDs(false, i);
      }
      RSReset = false;
    }
  }
}
