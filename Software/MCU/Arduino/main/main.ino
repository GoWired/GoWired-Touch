/*
 * GoWired is an open source project for WIRED home automation. It aims at making wired
 * home automation easy and affordable for every home automation enthusiast. GoWired provides:
 * - hardware (https://www.crowdsupply.com/domatic/getwired),
 * - software (https://github.com/GoWired/GoWired-Project/tree/master/Software), 
 * - 3D printable enclosures (https://github.com/GoWired/GoWired-Project/tree/master/Enclosures),
 * - instructions (both campaign page / campaign updates and our GitHub wiki).
 * 
 * GetWired is based on RS485 industrial communication standard. The software is an implementation
 * of MySensors communication protocol (http://www.mysensors.org). 
 *
 * Created by feanor-anglin
 * Copyright (C) 2018-2021 feanor-anglin
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 0.0.0 - feanor-anglin
 *
 * DESCRIPTION version 0.0.0
 * This is code for GoWired MCU Touch.
 * 
 * Hardware serial is used with baud rate of 57600 by default.
 * 
 * 
 */

/*  *******************************************************************************************
                                        Includes
 *  *******************************************************************************************/
#include "PowerSensor.h"
#include "InOut.h"
#include "Dimmer.h"
#include "RShutterControl.h"
#include "Configuration.h"
#include <MySensors.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "SHTSensor.h"
#include "LP50XX.h"

/*  *******************************************************************************************
                                        Globals
 *  *******************************************************************************************/
// General
// 0 - 2Relay; 1 - RGBW
uint8_t HardwareVariant;
// HardwareVariant = 0: LoadVariant=> 0 - 1 relay; 1 - 2 relays; 2 - roller shutter
// HardwareVariant = 1: LoadVariant=> 0 - rgb; 1 - rgbw; 2 - 1-channel dimmer
uint8_t LoadVariant;
uint8_t Iterations;

// RShutter
uint32_t MovementTime;
uint32_t StartTime;

// Timer
uint32_t LastUpdate = 0;               // Time of last update of interval sensors
bool CheckNow = true;

// Module Safety Indicators
bool OVERCURRENT_ERROR = false;            // Overcurrent error status
bool InformControllerES = false;            // Was controller informed about error?
bool ET_ERROR = 0;                       // External thermometer status (0 - ok, 1 - error)

// Initialization
bool InitConfirm = false;

// Touch Diagnosis
int TouchValueAggregated[2];
uint8_t LimitTransgressions = 0;

/*  *******************************************************************************************
                                        Constructors
 *  *******************************************************************************************/
// LP5009 - onboard RGB LED controller
LP50XX LP5009(BGR, LP5009_ENABLE_PIN);

// InOut class constructor
InOut IO[NUMBER_OF_RELAYS+NUMBER_OF_INPUTS];

// RShutterControl class Constructor
RShutterControl RS(RELAY_PIN_1, RELAY_PIN_2, RELAY_ON, RELAY_OFF);

// Dimmer class constructor
Dimmer Dimmer;

// Power sensor class constructor
#if defined(POWER_SENSOR)
  PowerSensor PS;
  MyMessage MsgWATT(0, V_WATT);
#endif

// SHTSensor class constructor
#ifdef SHT30
  SHTSensor sht;
  MyMessage MsgTEMP(0, V_TEMP);
  MyMessage MsgHUM(0, V_HUM);
#endif

// Protocol messages constructors
MyMessage MsgSTATUS(0, V_STATUS);
MyMessage MsgUP(0, V_UP);
MyMessage MsgDOWN(0, V_DOWN);
MyMessage MsgSTOP(0, V_STOP);
MyMessage MsgPERCENTAGE(0, V_PERCENTAGE);
MyMessage MsgRGB(0, V_RGB);
MyMessage MsgRGBW(0, V_RGBW);

// Debug
#ifdef RS485_DEBUG
  MyMessage MsgTEXT(0, V_TEXT);
  MyMessage MsgVAR1(0, V_VAR1);
#endif

/*  *******************************************************************************************
                                            Before
 *  *******************************************************************************************/
void before() {

  #ifdef ENABLE_WATCHDOG
    wdt_reset();
    MCUSR = 0;
    wdt_disable();
  #endif

  uint32_t InitDelay;

  if(_nodeId != AUTO) {
    InitDelay = _nodeId * INIT_DELAY;
  }
  else  {
    InitDelay = 0;
  }
  
  wait(InitDelay);
}

/*  *******************************************************************************************
                                            Setup
 *  *******************************************************************************************/
void setup() {

  // Resistive hardware detection
  pinMode(HARDWARE_DETECTION_PIN, INPUT_PULLUP);
  delay(100);

  uint16_t ReadHardware = analogRead(HARDWARE_DETECTION_PIN);

  if(ReadHardware < 20) {
    // Hardware variant: 2Relay Board
    HardwareVariant = 0;
  }
  else if(ReadHardware < 40)  {
    // Hardware variant: RGBW Board
    HardwareVariant = 1;
  }
  else  {/* Handling error */}

  // Reading dip switches, determining load variant
  pinMode(DIP_SWITCH_1, INPUT_PULLUP);
  pinMode(DIP_SWITCH_2, INPUT_PULLUP);
  pinMode(DIP_SWITCH_3, INPUT_PULLUP);

  delay(100);

  if(digitalRead(DIP_SWITCH_1) == false) {
    // DIP SWITCH 1 ON
    // HardwareVariant 0: roller shutter; HardwareVariant 1: Dimmer 
    LoadVariant = 2;
  }
  else  {
    if(digitalRead(DIP_SWITCH_2) == false)  {
      // DIP SWITCH 2 ON
      // HardwareVariant 0: single switch; HardwareVariant 1: RGB
      LoadVariant = 0;
    }
    else  {
      // Default option - DIP SWITCH 1 & DIP SWITCH 2 OFF
      // HardwareVariant 0: double switch; HardwareVariant 1: RGBW
      LoadVariant = 1;
    }
  }

  if(digitalRead(DIP_SWITCH_3) == false) {
    // Saving States to EEPROM
    // TO BE DEVELOPED
  }

  // Calculating number of iterations depending from IO instances
  Iterations = (HardwareVariant == 0 && LoadVariant == 0) ? 1 : 2;

  // Detecting I2C peripherals
  Wire.begin();
  // TO BE DEVELOPED

  // Support for 400kHz available
  //Wire.setClock(400000UL);

  // Initializing LP5009/LP5012
  LP5009.Begin();

  // LED: 0 || 1 || 2; R,G,B: 0-255, brightness: 0-255
  // LED0 - D1 (Touch Field A0), LED1 - D3 (Touch Field A2), LED2 - D2 (Touch Field A1)
  // LP5009.SetLEDColor(LED, R, G, B)
  // LP5009.SetLEDBrightness(LED, brightness)

  // LED initialization visual effect
  RainbowLED(RAINBOW_DURATION, RAINBOW_RATE);

  float Vcc = ReadVcc();  // mV

  // Initializing POWER SENSOR
  #if defined(POWER_SENSOR)
    PS.SetValues(PS_PIN, MVPERAMP, RECEIVER_VOLTAGE, MAX_CURRENT, POWER_MEASURING_TIME, Vcc);
  #endif

  // Initializing inputs & outputs
  if(HardwareVariant == 0)  {
    if(LoadVariant == 0)  {
      // Lighting: One button, single output
      IO[RELAY_ID_1].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_3, INPUT_PIN_1, RELAY_PIN_1);
    }
    else if(LoadVariant == 1) {
      // Lighting: two buttons, double output
      IO[RELAY_ID_1].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_1, INPUT_PIN_1, RELAY_PIN_1);
      IO[RELAY_ID_2].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_2, INPUT_PIN_2, RELAY_PIN_2);
    }
    else if(LoadVariant == 2) {
      // Roller shutter
      IO[RS_ID].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_1, INPUT_PIN_1, RELAY_PIN_1);
      IO[RS_ID + 1].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_2, INPUT_PIN_2, RELAY_PIN_2);
      // Temporary calibration
      RS.Calibration(UP_TIME, DOWN_TIME);
    }
  }
  else if(HardwareVariant == 1) {
    if(LoadVariant == 2)  {
      // 1-channel dimmer
      Dimmer.SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_W);
    }
    else if(LoadVariant == 0) {
      // RGB dimmer
      Dimmer.SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_R, LED_PIN_G, LED_PIN_B);
    }
    else if(LoadVariant == 1) {
      // RGBW dimmer
      Dimmer.SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_R, LED_PIN_G, LED_PIN_B, LED_PIN_W);
    }
    // Every dimmer has two buttons
    IO[0].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_1);
    IO[1].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_2);
  }

  if(HardwareVariant == 0 && LoadVariant == 0)  {
    /* AdjustLEDs(State [0 - OFF, 1 - ON, 2 - Rainbow], Button [0 or 1] */
    AdjustLEDs(0, 0);
  }
  else  {
    AdjustLEDs(0, 0);
    AdjustLEDs(0, 1);
  }

  // ONBOARD THERMOMETER
  #ifdef SHT30
    sht.init();
    sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
  #endif

  // Enable AVR Watchdog
  #ifdef ENABLE_WATCHDOG
    wdt_enable(WDTO_2S);
  #endif

}

/*  *******************************************************************************************
                                            Presentation
 *  *******************************************************************************************/
void presentation() {

  sendSketchInfo(SN, SV);

  // OUTPUT
  if(HardwareVariant == 0)  {
    if(LoadVariant == 0)  {
      present(RELAY_ID_1, S_BINARY, "Relay 1");   wait(PRESENTATION_DELAY);
    }
    else if(LoadVariant == 1) {
      present(RELAY_ID_1, S_BINARY, "Relay 1");   wait(PRESENTATION_DELAY);
      present(RELAY_ID_2, S_BINARY, "Relay 2");   wait(PRESENTATION_DELAY);
    }
    else if(LoadVariant == 2) {
      present(RS_ID, S_COVER, "Roller Shutter");  wait(PRESENTATION_DELAY);     
    }
  }
  else if(HardwareVariant == 1) {
    if(LoadVariant == 2)  {
      present(DIMMER_ID, S_DIMMER, "Dimmer"); wait(PRESENTATION_DELAY);
    }
    else if(LoadVariant == 0) {
      present(DIMMER_ID, S_RGB_LIGHT, "RGB"); wait(PRESENTATION_DELAY);
    }
    else if(LoadVariant == 1) {
      present(DIMMER_ID, S_RGBW_LIGHT, "RGBW");   wait(PRESENTATION_DELAY);
    }
  }

  #ifdef SPECIAL_BUTTON
    present(SPECIAL_BUTTON_ID, S_BINARY, "Special Button"); wait(PRESENTATION_DELAY);
  #endif

  // POWER SENSOR
  #if defined(POWER_SENSOR)
    present(PS_ID, S_POWER, "Power Sensor");    wait(PRESENTATION_DELAY);
  #endif

  // Onboard Thermometer
  #ifdef SHT30
    present(ETT_ID, S_TEMP, "External Thermometer"); wait(PRESENTATION_DELAY);
    present(ETH_ID, S_HUM, "External Hygrometer");  wait(PRESENTATION_DELAY);
  #endif

  // Electronic fuse
  #ifdef ELECTRONIC_FUSE 
      present(ES_ID, S_BINARY, "OVERCURRENT ERROR");    wait(PRESENTATION_DELAY);
  #endif

  #ifdef SHT30
    present(ETS_ID, S_BINARY, "ET STATUS");   wait(PRESENTATION_DELAY);
  #endif

  #ifdef RS485_DEBUG
    present(DEBUG_ID, S_INFO, "DEBUG INFO");
    present(TOUCH_DIAGNOSTIC_ID, S_DISTANCE, "Touch Diagnostic");
  #endif

}

/*  *******************************************************************************************
                                            Init Confirmation
 *  *******************************************************************************************/
void InitConfirmation() {

  // OUTPUT
  // 2Relay
  if(HardwareVariant == 0)  {
    // Single output
    if(LoadVariant == 0)  {
      send(MsgSTATUS.setSensor(RELAY_ID_1).set(IO[RELAY_ID_1].NewState));
      request(RELAY_ID_1, V_STATUS);
      wait(2000, C_SET, V_STATUS);
    }
    // Double output
    else if(LoadVariant == 1) {
      send(MsgSTATUS.setSensor(RELAY_ID_1).set(IO[RELAY_ID_1].NewState));
      request(RELAY_ID_1, V_STATUS);
      wait(2000, C_SET, V_STATUS);

      send(MsgSTATUS.setSensor(RELAY_ID_2).set(IO[RELAY_ID_2].NewState));
      request(RELAY_ID_2, V_STATUS);
      wait(2000, C_SET, V_STATUS);
    }
    // Roller shutter
    else if(LoadVariant == 2) {
      send(MsgUP.setSensor(RS_ID).set(0));
      request(RS_ID, V_UP);
      wait(2000, C_SET, V_UP);

      send(MsgDOWN.setSensor(RS_ID).set(0));
      request(RS_ID, V_DOWN);
      wait(2000, C_SET, V_DOWN);

      send(MsgSTOP.setSensor(RS_ID).set(0));
      request(RS_ID, V_STOP);
      wait(2000, C_SET, V_STOP);

      send(MsgPERCENTAGE.setSensor(RS_ID).set(RS.Position));
      request(RS_ID, V_PERCENTAGE);
      wait(2000, C_SET, V_PERCENTAGE);
    }
  }
  // RGBW
  else if(HardwareVariant == 1) {
    // RGB dimmer
    if(LoadVariant == 0)  {
      send(MsgSTATUS.setSensor(DIMMER_ID).set(false));
      request(DIMMER_ID, V_STATUS);
      wait(2000, C_SET, V_STATUS);
    
      send(MsgPERCENTAGE.setSensor(DIMMER_ID).set(0));
      request(DIMMER_ID, V_PERCENTAGE);
      wait(2000, C_SET, V_PERCENTAGE);

      send(MsgRGB.setSensor(DIMMER_ID).set("000000"));
      request(DIMMER_ID, V_RGB);
      wait(2000, C_SET, V_RGB);
    }
    // RGBW dimmer
    else if(LoadVariant == 1) {
      send(MsgSTATUS.setSensor(DIMMER_ID).set(false));
      request(DIMMER_ID, V_STATUS);
      wait(2000, C_SET, V_STATUS);

      send(MsgPERCENTAGE.setSensor(DIMMER_ID).set(0));
      request(DIMMER_ID, V_PERCENTAGE);
      wait(2000, C_SET, V_PERCENTAGE);

      send(MsgRGBW.setSensor(DIMMER_ID).set("00000000"));
      request(DIMMER_ID, V_RGBW);
      wait(2000, C_SET, V_RGBW);
    }
    // 1-channel dimmer
    else if(LoadVariant == 2) {
      send(MsgSTATUS.setSensor(DIMMER_ID).set(false));
      request(DIMMER_ID, V_STATUS);
      wait(2000, C_SET, V_STATUS);

      send(MsgPERCENTAGE.setSensor(DIMMER_ID).set(0));
      request(DIMMER_ID, V_PERCENTAGE);
      wait(2000, C_SET, V_PERCENTAGE);
    }
  }

  #ifdef SPECIAL_BUTTON
    send(MsgSTATUS.setSensor(SPECIAL_BUTTON_ID).set(0));
  #endif

  // Built-in sensors
  #ifdef POWER_SENSOR
    send(MsgWATT.set("0"));
  #endif

  // External sensors
  #ifdef SHT30
    ETUpdate();
    send(MsgSTATUS.setSensor(ETS_ID).set(0));
  #endif

  //
  #ifdef ELECTRONIC FUSE
    send(MsgSTATUS.setSensor(ES_ID).set(0));
  #endif

  #ifdef RS485_DEBUG
    send(MsgTEXT.setSensor(DEBUG_ID).set("DEBUG MESSAGE"));
    send(MsgVAR1.setSensor(TOUCH_DIAGNOSTIC_ID).set(0));
  #endif

  InitConfirm = true;

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

    if(HardwareVariant == 0 && LoadVariant == 0)  {
      LP5009.SetLEDColor(BUILTIN_LED3, RValue, GValue, BValue);
      LP5009.SetLEDBrightness(BUILTIN_LED3, BRIGHTNESS_VALUE_ON);
    }
    else {
      uint8_t LEDs[2] = {BUILTIN_LED1, BUILTIN_LED2};
      for(int i=0; i<2; i++)  {
        LP5009.SetLEDColor(LEDs[i], RValue, GValue, BValue);
        LP5009.SetLEDBrightness(LEDs[i], BRIGHTNESS_VALUE_ON);
      }
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
    wait(Rate);
  }
}

// Adjust builtin LEDs according to output states
void AdjustLEDs(uint8_t State, uint8_t Button) {

  uint8_t LED;

  if(HardwareVariant == 0 && LoadVariant == 0)  {
    LED = BUILTIN_LED3;
  }
  else  {
    LED = Button == 0 ? BUILTIN_LED1 : BUILTIN_LED2;
  }

  switch(State) {
    case 0:
      LP5009.SetLEDColor(LED, R_VALUE_OFF, G_VALUE_OFF, B_VALUE_OFF);
      LP5009.SetLEDBrightness(LED, BRIGHTNESS_VALUE_OFF);
      break;
    case 1:
      LP5009.SetLEDColor(LED, R_VALUE_ON, G_VALUE_ON, B_VALUE_ON);
      LP5009.SetLEDBrightness(LED, BRIGHTNESS_VALUE_ON);
      break;
    case 2:
      RainbowLED(RAINBOW_DURATION, RAINBOW_RATE);
      break;
    default:
      break;
  }
}

// Adjust builtin LEDs by color & brightness
void AdjustLEDs2(uint8_t LED, uint8_t Brightness, uint8_t R, uint8_t G, uint8_t B) {

  // LED: BUILTIN_LED1 / BUILTIN_LED2 / BUILTIN_LED3
  LP5009.SetLEDColor(LED, R, G, B);
  LP5009.SetLEDBrightness(LED, Brightness);
}

// Touch diagnosis
bool TouchDiagnosis(uint16_t Threshold) {

  for(int i=0; i<Iterations; i++)  {
    if(TouchValueAggregated[i] > (int)(Threshold / 2) || TouchValueAggregated[i] < -(int)(Threshold / 2))  {
      
      #ifdef RS485_DEBUG
        // Send message with TouchValue
        send(MsgVAR1.setSensor(TOUCH_DIAGNOSTIC_ID).set(TouchValueAggregated[i]));
      #endif

      if(TouchValueAggregated[i] < Threshold) {
        LimitTransgressions++;
      }
    }
  }
}

// Reading new reference for touch buttons
void ReadNewReference() {

  for(int i=0; i<Iterations; i++)  {
    // Turn off LEDs
    for(int j=1; j<4; j++)  {
      LP5009.SetLEDBrightness(j, 0);
    }
    // Take a break from running
    delay(1000);
    // Read new reference
    IO[i].ReadReference();
    // Rainbow LED visual effect
    RainbowLED(RAINBOW_DURATION, RAINBOW_RATE);
    // Turn LEDs on
    AdjustLEDs(IO[i].OldState, i);
  }
}

/*  *******************************************************************************************
                                        MySensors Receive
 *  *******************************************************************************************/
void receive(const MyMessage &message)  {
  
  // Binary messages
  if (message.type == V_STATUS) {
    #ifdef SPECIAL_BUTTON
      if (message.sensor == SPECIAL_BUTTON_ID)  {
        // Ignore this message
      }
    #endif
    if(HardwareVariant == 1)  {
      if (message.sensor == DIMMER_ID) {
        Dimmer.NewState = message.getBool();
        for(int i=0; i<2; i++)  {
          AdjustLEDs(Dimmer.NewState, i);
        }
        Dimmer.ChangeState();
      }
    }
    if(HardwareVariant == 0 && LoadVariant != 2)  {
      if (message.sensor == RELAY_ID_1 || message.sensor == RELAY_ID_2)  {
        if (!OVERCURRENT_ERROR) {
          IO[message.sensor].NewState = message.getBool();
          IO[message.sensor].SetRelay();
          AdjustLEDs(IO[message.sensor].NewState, message.sensor);
        }
      }
    }
    // Secret configuration
    if(message.sensor == SECRET_CONFIG_ID_1)  {
      // Roller shutter calibration
      float Vcc = ReadVcc();
      RSCalibration(Vcc);
    }
  }
  // Percentage messages
  else if (message.type == V_PERCENTAGE) {
    #ifdef ROLLER_SHUTTER
      if(message.sensor == RS_ID) {
        int NewPosition = atoi(message.data);
        NewPosition = NewPosition > 100 ? 100 : NewPosition;
        NewPosition = NewPosition < 0 ? 0 : NewPosition;
        RS.NewState = 2;
        RSUpdate();
        MovementTime = RS.ReadNewPosition(NewPosition);
      }
    #endif
    #if defined(DIMMER) || defined(RGB) || defined(RGBW)
      if(message.sensor == DIMMER_ID) {
        Dimmer.NewDimmingLevel = atoi(message.data);
        Dimmer.NewDimmingLevel = Dimmer.NewDimmingLevel > 100 ? 100 : Dimmer.NewDimmingLevel;
        Dimmer.NewDimmingLevel = Dimmer.NewDimmingLevel < 0 ? 0 : Dimmer.NewDimmingLevel;

        Dimmer.NewState = true;
        for(int i=0; i<2; i++)  {
          AdjustLEDs(Dimmer.NewState, i);
        }
        Dimmer.ChangeLevel();
      }
    #endif
  }
  // RGB/RGBW messages
  else if (message.type == V_RGB || message.type == V_RGBW) {
    #if defined(RGB) || defined(RGBW)
      if(message.sensor == DIMMER_ID) {
        const char *rgbvalues = message.getString();

        Dimmer.NewState = true;
        for(int i=0; i<2; i++)  {
          AdjustLEDs(Dimmer.NewState, i);
        }
        Dimmer.NewColorValues(rgbvalues);
        Dimmer.ChangeColors();
      }
    #endif
  }
  // Roller shutter control messages (UP, DOWN, STOP)
  else if(message.type == V_UP) {
    #ifdef ROLLER_SHUTTER
      if(message.sensor == RS_ID) {
        MovementTime = RS.ReadMessage(0);
      }
    #endif
  }
  else if(message.type == V_DOWN) {
    #ifdef ROLLER_SHUTTER
      if(message.sensor == RS_ID) {
        MovementTime = RS.ReadMessage(1);
      }
    #endif
  }
  else if(message.type == V_STOP) {
    #ifdef ROLLER_SHUTTER
      if(message.sensor == RS_ID) {
        MovementTime = RS.ReadMessage(2);
      }
    #endif
  }
}

/*  *******************************************************************************************
                                      External Thermometer
 *  *******************************************************************************************/
void ETUpdate()  {

  #ifdef SHT30
    if(sht.readSample())  {
      ET_ERROR = 0;
      send(MsgSTATUS.setSensor(ETS_ID).set(ET_ERROR));
      send(MsgTEMP.setSensor(ETT_ID).setDestination(0).set(sht.getTemperature(), 1));
      send(MsgHUM.setSensor(ETH_ID).set(sht.getHumidity(), 1));
      #ifdef HEATING_SECTION_SENSOR
        send(MsgTEMP.setSensor(ETT_ID).setDestination(MY_HEATING_CONTROLLER).set(sht.getTemperature(), 1));
      #endif
    }
    else  {
      ET_ERROR = 1;
      send(MsgSTATUS.setSensor(ETS_ID).set(ET_ERROR));
    }
  #endif
}

/*  *******************************************************************************************
                                        IO Update
 *  *******************************************************************************************/
void IOUpdate() {

  uint8_t FirstSensor = 0;
  uint8_t LongpressDetectionCount = 0; 

  if (Iterations > 0)  {
    for (int i = FirstSensor; i < FirstSensor + Iterations; i++)  {
      TouchValueAggregated[i] = IO[i].ReadInput(TOUCH_THRESHOLD, LONGPRESS_DURATION, DEBOUNCE_VALUE);
      if (IO[i].NewState != IO[i].OldState)  {
        switch(IO[i].SensorType)  {
          case 0:
            // Touch Fields only (Hardware: RGBW)
            if(HardwareVariant == 1)  {
              if(i == 0)  {
                if(IO[i].NewState != 2) {
                  // Change dimmer status
                  Dimmer.NewState = !Dimmer.NewState;
                  for(int j=0; j<2; j++)  {
                    AdjustLEDs(IO[i].NewState, j);
                  }
                  Dimmer.ChangeState();
                  send(MsgSTATUS.setSensor(DIMMER_ID).set(Dimmer.NewState));
                  IO[i].OldState = IO[i].NewState;
                }
                #ifdef SPECIAL_BUTTON
                  if(IO[i].NewState == 2) {
                    AdjustLEDs(IO[i].NewState, 0);
                    IO[i].NewState = IO[i].OldState;
                    AdjustLEDs(IO[i].NewState, 0);
                    int j = i == 0 ? 1 : 0;
                    AdjustLEDs(IO[j].NewState, j);
                    LongpressDetectionCount++;
                  }
                #endif  
              }
              else if(i == 1) {
                if(IO[i].NewState != 2)  {
                  if(Dimmer.NewState) {
                    // Toggle dimming level by DIMMING_TOGGLE_STEP
                    Dimmer.NewDimmingLevel += DIMMING_TOGGLE_STEP;

                    Dimmer.NewDimmingLevel = Dimmer.NewDimmingLevel > 100 ? DIMMING_TOGGLE_STEP : Dimmer.NewDimmingLevel;
                    send(MsgPERCENTAGE.setSensor(DIMMER_ID).set(Dimmer.NewDimmingLevel));
                    Dimmer.ChangeLevel();
                  }
                  else  {
                    // If dimmer is turned off - turn it on
                    Dimmer.NewState = 1;
                    for(int j=0; j<2; j++)  {
                      AdjustLEDs(IO[i].NewState, j);
                    }
                    Dimmer.ChangeState();
                    send(MsgSTATUS.setSensor(DIMMER_ID).set(Dimmer.NewState));
                    IO[i].OldState = IO[i].NewState;
                  }
                }
                else  {
                  // Handling of longpress of the second button
                }
              }
            }
          case 1:
            // Touch Fields & External buttons (Hardware: 2Relay)
            if(HardwareVariant == 0)  {
              // 1 relay or 2 relays
              if(LoadVariant == 0 || LoadVariant == 1)  {
                if (IO[i].NewState != 2)  {
                  if (!OVERCURRENT_ERROR)  {
                    IO[i].SetRelay();
                    AdjustLEDs(IO[i].NewState, i);
                    send(MsgSTATUS.setSensor(i).set(IO[i].NewState));
                  }
                }
              }
              // Roller shutter
              else if(LoadVariant == 2)  {
                if(IO[i].NewState != 2)  {
                  MovementTime = RS.ReadButtons(i);
                  // Adjusting LEDs is done in RSUpdate() function
                  IO[i].OldState = IO[i].NewState;
                }
              }
            }
            // Special button
            if(IO[i].NewState == 2) {
              #ifdef SPECIAL_BUTTON
                AdjustLEDs(IO[i].NewState, i);
                IO[i].NewState = IO[i].OldState;
                AdjustLEDs(IO[i].NewState, i);
                if(LoadVariant != 0)  {
                  int j = i == 0 ? 1 : 0;
                  AdjustLEDs(IO[j].NewState, j);
                }
                LongpressDetectionCount++;
              #endif
            }
            break;
          default:
            // Nothing to do here
          break;
        }
      }
    }
    if(LongpressDetectionCount == 1)  {
      #ifdef SPECIAL_BUTTON
        send(MsgSTATUS.setSensor(SPECIAL_BUTTON_ID).set(true));
      #endif
    }
    else if(LongpressDetectionCount > 1) {
      #ifdef TOUCH_AUTO_DIAGNOSTICS
        ReadNewReference();
      #endif
    }
  }
}

/*  *******************************************************************************************
                                    Roller Shutter Calibration
 *  *******************************************************************************************/
void RSCalibration(float Vcc)  {

  float Current = 0;
  uint16_t DownTimeCumulated = 0;
  uint16_t UpTimeCumulated = 0;
  uint32_t StartTime = 0;
  uint32_t StopTime = 0;
  uint32_t MeasuredTime = 0;

  // Opening the shutter  
  RS.NewState = 0;
  RS.Movement();

  do  {
    delay(100);
    wdt_reset();
    Current = PS.MeasureAC(Vcc);
  } while(Current > PS_OFFSET);

  RS.NewState = 2;
  RS.Movement();

  delay(1000);

  // Calibrating
  for(int i=0; i<CALIBRATION_SAMPLES; i++) {
    for(int j=1; j>=0; j--)  {
      RS.NewState = j;
      RS.Movement();
      StartTime = millis();

      do  {
        delay(100);
        Current = PS.MeasureAC(Vcc);
        StopTime = millis();
        wdt_reset();
      } while(Current > PS_OFFSET);

      RS.NewState = 2;
      RS.Movement();

      MeasuredTime = StopTime - StartTime;

      if(j) {
        DownTimeCumulated += (int)(MeasuredTime / 1000);
      }
      else  {
        UpTimeCumulated += (int)(MeasuredTime / 1000);
      }

      delay(1000);
    }
  }

  RS.Position = 0;

  uint8_t DownTime = (int)(DownTimeCumulated / CALIBRATION_SAMPLES);
  uint8_t UpTime = (int)(UpTimeCumulated / CALIBRATION_SAMPLES);

  if(DownTime > 1 && UpTime > 1)  {
    RS.Calibration(UpTime, DownTime);
  }
}

/*  *******************************************************************************************
                                        Roller Shutter
 *  *******************************************************************************************/
void RSUpdate() {

  uint32_t StopTime = 0;
  uint32_t MeasuredTime;
  bool Direction;

  if(RS.State != RS.NewState) {
    if(RS.NewState != 2)  {
      RS.Movement();
      StartTime = millis();
      if(RS.NewState == 0)  {
        AdjustLEDs(1, 0);
        send(MsgUP.setSensor(RS_ID));
      }
      else if(RS.NewState == 1) {
        AdjustLEDs(1, 1);
        send(MsgDOWN.setSensor(RS_ID));
      }
    }
    else  {
      Direction = RS.State;
      RS.Movement();
      StopTime = millis();
      AdjustLEDs(0, 0);
      AdjustLEDs(0, 1);
      send(MsgSTOP.setSensor(RS_ID));
    }
  }

  if(RS.State != 2) {
    if(millis() >= StartTime + MovementTime) {
      Direction = RS.State;
      RS.NewState = 2;
      RS.Movement();
      StopTime = millis();
      AdjustLEDs(0, 0);
      AdjustLEDs(0, 1);
      send(MsgSTOP.setSensor(RS_ID));
    }
    if(millis() < StartTime)  {
      uint32_t Temp = 4294967295 - StartTime + millis();
      wait(MovementTime - Temp);
      RS.NewState = 2;
      RS.Movement();
      AdjustLEDs(0, 0);
      AdjustLEDs(0, 1);
      send(MsgSTOP.setSensor(RS_ID));
      StartTime = 0;
      StopTime = MovementTime;
    }
  }

  if(StopTime > 0)  {
    MeasuredTime = StopTime - StartTime;
    RS.CalculatePosition(Direction, MeasuredTime);
  
    send(MsgPERCENTAGE.setSensor(RS_ID).set(RS.Position));
  }
}

/*  *******************************************************************************************
                                        Power Sensor
 *  *******************************************************************************************/
void PSUpdate(float Current, uint8_t Sensor = 0)  {
  
  #if defined(POWER_SENSOR)
    send(MsgWATT.setSensor(PS_ID).set(PS.CalculatePower(Current, COSFI), 0));
    PS.OldValue = Current;
  #endif

}

/*  *******************************************************************************************
 *                                    Read Vcc
 *  *******************************************************************************************/
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
}

/*  *******************************************************************************************
                                        Main Loop
 *  *******************************************************************************************/
void loop() {

  float Vcc = ReadVcc(); // mV
  float Current = 0;

  // Sending out states for the first time (as required by Home Assistant)
  if (!InitConfirm)  {
    InitConfirmation();
  }

  // Reading inputs / activating outputs
  IOUpdate();

  #ifdef TOUCH_AUTO_DIAGNOSTICS
    // Checking if touch feature works correctly
    TouchDiagnosis(TOUCH_THRESHOLD);

    // Reading new touch reference if diagnosis shows that previous values were not correct
    if(LimitTransgressions > TOUCH_DIAG_TRESHOLD)  {
      ReadNewReference();
      LimitTransgressions = 0;
    }
  #endif

  // Updating roller shutter
  if(HardwareVariant == 0 && LoadVariant == 2)  {
    RSUpdate();
  }

  // Reading power sensor(s)
  #ifdef POWER_SENSOR
    // 2Relay Board
    if(HardwareVariant == 0)  {
      if (digitalRead(RELAY_PIN_1) == RELAY_ON || digitalRead(RELAY_PIN_2) == RELAY_ON)  {
        Current = PS.MeasureAC(Vcc);
      }
    }
    // RGBW Board
    if(HardwareVariant == 1)  {
      if (Dimmer.NewState)  {
        Current = PS.MeasureDC(Vcc);
      }
    }
      
    #ifdef ERROR_REPORTING
      OVERCURRENT_ERROR = PS.ElectricalStatus(Current);
    #endif
    
    if (Current < 0.5) {
      if (Current == 0 && PS.OldValue != 0)  {
        PSUpdate(Current);
      }
      else if (abs(PS.OldValue - Current) > 0.05) {
        PSUpdate(Current);
      }
    }
    else  {
      if(abs(PS.OldValue - Current) > (0.1 * PS.OldValue))  {
        PSUpdate(Current);
      }
    }
  #endif

  // Current safety
  #if defined(ELECTRONIC_FUSE) && defined(POWER_SENSOR)
    if(OVERCURRENT_ERROR)  {
      // Current to high
      // Board: 2Relay 
      if(HardwareVariant == 0)  {
        // Load: Lighting
        if(LoadVariant < 2) {
          for (int i = RELAY_ID_1; i < RELAY_ID_1 + NUMBER_OF_RELAYS; i++)  {
            IO[i].NewState = RELAY_OFF;
            IO[i].SetRelay();
            send(MsgSTATUS.setSensor(i).set(IO[i].NewState));
          }
        }
        // Load: Roller shutter
        else if(LoadVariant == 2) {
          RS.NewState = 2;
          RSUpdate();
        }
      }
      // Board: RGBW
      else if(HardwareVariant == 1) {
        Dimmer.NewState = false;
        Dimmer.ChangeState();
        send(MsgSTATUS.setSensor(DIMMER_ID).set(Dimmer.NewState));
      }
      send(MsgSTATUS.setSensor(ES_ID).set(OVERCURRENT_ERROR));
      InformControllerES = true;
    }
    else if(!OVERCURRENT_ERROR && InformControllerES)  {
      // Current normal (only after reporting error)
      send(MsgSTATUS.setSensor(ES_ID).set(OVERCURRENT_ERROR));
      InformControllerES = false;
    }
  #endif

  // Reset LastUpdate if millis() has overflowed
  if(LastUpdate > millis()) {
    LastUpdate = millis();
  }  
  
  // Checking out sensors which report at a defined interval
  if ((millis() > LastUpdate + INTERVAL) || CheckNow == true)  {
    #ifdef SHT30
      ETUpdate();
    #endif
    LimitTransgressions = 0;
    LastUpdate = millis();
    CheckNow = false;
  }

  wait(LOOP_TIME);
}
/*

   EOF

*/
