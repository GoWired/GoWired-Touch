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
// RShutter
#ifdef ROLLER_SHUTTER
  uint32_t MovementTime;
  uint32_t StartTime;
#endif

// Timer
uint32_t LastUpdate = 0;               // Time of last update of interval sensors
bool CheckNow = false;

// Module Safety Indicators
bool OVERCURRENT_ERROR = false;            // Overcurrent error status
bool InformControllerES = false;            // Was controller informed about error?
bool ET_ERROR = 0;                       // External thermometer status (0 - ok, 1 - error)

// Initialization
bool InitConfirm = false;

/*  *******************************************************************************************
                                        Constructors
 *  *******************************************************************************************/
// LP5009 - onboard RGB LED controller
LP50XX LP5009(BGR, LP5009_ENABLE_PIN);

//Universal input constructor
InOut IO[NUMBER_OF_RELAYS+NUMBER_OF_INPUTS];
MyMessage msgIO(0, V_LIGHT);

// RShutter Control Constructor
#ifdef ROLLER_SHUTTER
  RShutterControl RS(RELAY_1, RELAY_2, RELAY_ON, RELAY_OFF);
  MyMessage msgRS1(RS_ID, V_UP);
  MyMessage msgRS2(RS_ID, V_DOWN);
  MyMessage msgRS3(RS_ID, V_STOP);
  MyMessage msgRS4(RS_ID, V_PERCENTAGE);
#endif

// Dimmer
#if defined(DIMMER) || defined(RGB) || defined(RGBW)
  Dimmer Dimmer;
  MyMessage msgDIM(DIMMER_ID, V_PERCENTAGE);
  MyMessage msgDIM2(DIMMER_ID, V_RGB);
  MyMessage msgDIM3(DIMMER_ID, V_RGBW);
#endif

// Power sensor
#if defined(POWER_SENSOR)
  PowerSensor PS;
  MyMessage msgPS(PS_ID, V_WATT);
#endif

// Onboard thermometer
#ifdef SHT30
  SHTSensor sht;
  MyMessage msgETT(ETT_ID, V_TEMP);
  MyMessage msgETH(ETH_ID, V_HUM);
#endif

// Error Reporting
#ifdef ELECTRONIC_FUSE
  MyMessage msgSI(0, V_STATUS);
#endif

#ifdef RS485_DEBUG
  MyMessage msgDEBUG(DEBUG_ID, V_TEXT);
  MyMessage msgDEBUG2(DEBUG_ID, V_WATT);
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

  uint32_t InitDelay = MY_NODE_ID * INIT_DELAY;
  
  wait(InitDelay);
}

/*  *******************************************************************************************
                                            Setup
 *  *******************************************************************************************/
void setup() {

  #ifdef ENABLE_WATCHDOG
    wdt_enable(WDTO_2S);
  #endif

  Wire.begin();

  // Support for 400kHz available
  //Wire.setClock(400000UL);

  LP5009.Begin();

  // LED: 0 || 1 || 2; R,G,B: 0-255, brightness: 0-255
  // LED0 - D1 (Touch Field A0), LED1 - D3 (Touch Field A2), LED2 - D2 (Touch Field A1)
  // LP5009.SetLEDColor(LED, R, G, B)
  // LP5009.SetLEDBrightness(LED, brightness)

  // LED initialization visual effect
  uint8_t TEMP[3] = {LED0, LED1, LED2};
  for(int i=0; i<256; i++)  {
    for(int j=0; j<3; j++)  {
      LP5009.SetLEDColor(TEMP[j], i, 0, 0);
      LP5009.SetLEDBrightness(TEMP[j], i);
      delay(10);
    }
  }
  for(int i=255; i>=0; i--)  {
    for(int j=0; j<3; j++)  {
      LP5009.SetLEDColor(TEMP[j], i, 0, 0);
      LP5009.SetLEDBrightness(TEMP[j], i);
      delay(10);
    }
  }
  for(int i=0; i<256; i++)  {
    for(int j=0; j<3; j++)  {
      LP5009.SetLEDColor(TEMP[j], 0, i, 0);
      LP5009.SetLEDBrightness(TEMP[j], i);
      delay(10);
    }
  }
  for(int i=255; i>=0; i--)  {
    for(int j=0; j<3; j++)  {
      LP5009.SetLEDColor(TEMP[j], 0, i, 0);
      LP5009.SetLEDBrightness(TEMP[j], i);
      delay(10);
    }
  }
  for(int i=0; i<256; i++)  {
    for(int j=0; j<3; j++)  {
      LP5009.SetLEDColor(TEMP[j], 0, 0, i);
      LP5009.SetLEDBrightness(TEMP[j], i);
      delay(10);
    }
  }
  for(int i=255; i>=0; i--)  {
    for(int j=0; j<3; j++)  {
      LP5009.SetLEDColor(TEMP[j], 0, 0, i);
      LP5009.SetLEDBrightness(TEMP[j], i);
      delay(10);
    }
  }

  float Vcc = ReadVcc();  // mV

  // POWER SENSOR
  #if defined(POWER_SENSOR)
    PS.SetValues(PS_PIN, MVPERAMP, RECEIVER_VOLTAGE, MAX_CURRENT, POWER_MEASURING_TIME, Vcc);
  #endif

  // OUTPUT
  #ifdef SINGLE_RELAY
    IO[RELAY_ID_1].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_1, INPUT_PIN_1, RELAY_PIN_1);
  #endif

  #ifdef DOUBLE_RELAY
    IO[RELAY_ID_1].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_1, INPUT_PIN_1, RELAY_PIN_1);
    IO[RELAY_ID_2].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_2, INPUT_PIN_2, RELAY_PIN_2);
  #endif

  #ifdef ROLLER_SHUTTER
    IO[RS_ID].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_1, INPUT_PIN_1, RELAY_PIN_1);
    IO[RS_ID + 1].SetValues(RELAY_OFF, RELAY_ON, 1, TOUCH_FIELD_2, INPUT_PIN_2, RELAY_PIN_2);
    if(!RS.Calibrated)  {
    #ifdef RS_AUTO_CALIBRATION
      RSCalibration(Vcc);
    #else
      RS.Calibration(UP_TIME, DOWN_TIME);
    #endif
    }
  #endif

  #ifdef DIMMER
    Dimmer.SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_W);
    IO[0].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_1);
    IO[1].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_2);
  #endif

  #ifdef RGB
    Dimmer.SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_R, LED_PIN_G, LED_PIN_B);
    IO[0].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_1);
    IO[1].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_2);
  #endif

  #ifdef RGBW
    Dimmer.SetValues(NUMBER_OF_CHANNELS, DIMMING_STEP, DIMMING_INTERVAL, LED_PIN_R, LED_PIN_G, LED_PIN_B, LED_PIN_W);
    IO[0].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_1);
    IO[1].SetValues(RELAY_OFF, RELAY_ON, 0, TOUCH_FIELD_2);
  #endif

  #ifdef SINGLE_RELAY
    LP5009.SetLEDColor(BUTTON_LED_1, R_VALUE_OFF, G_VALUE_OFF, B_VALUE_OFF);
    LP5009.SetLEDBrightness(BUTTON_LED_1, BRIGHTNESS_VALUE_OFF);
  #else
    LP5009.SetLEDColor(BUTTON_LED_1, R_VALUE_OFF, G_VALUE_OFF, B_VALUE_OFF);
    LP5009.SetLEDBrightness(BUTTON_LED_1, BRIGHTNESS_VALUE_OFF);
    LP5009.SetLEDColor(BUTTON_LED_2, R_VALUE_OFF, G_VALUE_OFF, B_VALUE_OFF);
    LP5009.SetLEDBrightness(BUTTON_LED_2, BRIGHTNESS_VALUE_OFF);
  #endif

  // ONBOARD THERMOMETER
  #ifdef SHT30
    Wire.begin();
    sht.init();
    sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
  #endif

}

/*  *******************************************************************************************
                                            Presentation
 *  *******************************************************************************************/
void presentation() {

  sendSketchInfo(SN, SV);

  // OUTPUT
  #ifdef SINGLE_RELAY
    present(RELAY_ID_1, S_BINARY, "Relay 1");   wait(PRESENTATION_DELAY);
  #endif

  #ifdef DOUBLE_RELAY
    present(RELAY_ID_1, S_BINARY, "Relay 1");   wait(PRESENTATION_DELAY);
    present(RELAY_ID_2, S_BINARY, "Relay 2");   wait(PRESENTATION_DELAY);
  #endif

  #ifdef ROLLER_SHUTTER
    present(RS_ID, S_COVER, "Roller Shutter");  wait(PRESENTATION_DELAY);
  #endif

  #ifdef DIMMER
    present(DIMMER_ID, S_DIMMER, "Dimmer"); wait(PRESENTATION_DELAY);
  #endif

  #ifdef RGB
    present(DIMMER_ID, S_RGB_LIGHT, "RGB"); wait(PRESENTATION_DELAY);
  #endif

  #ifdef RGBW
    present(DIMMER_ID, S_RGBW_LIGHT, "RGBW");   wait(PRESENTATION_DELAY);
  #endif

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

  // I2C


  // Electronic fuse
  #ifdef ELECTRONIC_FUSE 
      present(ES_ID, S_BINARY, "OVERCURRENT ERROR");    wait(PRESENTATION_DELAY);
  #endif

  #ifdef SHT30
    present(ETS_ID, S_BINARY, "ET STATUS");   wait(PRESENTATION_DELAY);
  #endif

  #ifdef RS485_DEBUG
    present(DEBUG_ID, S_INFO, "DEBUG INFO");
  #endif

}

/*  *******************************************************************************************
                                            Init Confirmation
 *  *******************************************************************************************/
void InitConfirmation() {

  // OUTPUT
  #ifdef SINGLE_RELAY
    send(msgIO.setSensor(RELAY_ID_1).set(IO[RELAY_ID_1].NewState));
    request(RELAY_ID_1, V_STATUS);
    wait(2000, C_SET, V_STATUS);
  #endif

  #ifdef DOUBLE_RELAY
    send(msgIO.setSensor(RELAY_ID_1).set(IO[RELAY_ID_1].NewState));
    request(RELAY_ID_1, V_STATUS);
    wait(2000, C_SET, V_STATUS);

    send(msgIO.setSensor(RELAY_ID_2).set(IO[RELAY_ID_2].NewState));
    request(RELAY_ID_2, V_STATUS);
    wait(2000, C_SET, V_STATUS);

  #endif

  #ifdef ROLLER_SHUTTER
    send(msgRS1.set(0));
    request(RS_ID, V_UP);
    wait(2000, C_SET, V_UP);

    send(msgRS2.set(0));
    request(RS_ID, V_DOWN);
    wait(2000, C_SET, V_DOWN);

    send(msgRS3.set(0));
    request(RS_ID, V_STOP);
    wait(2000, C_SET, V_STOP);

    send(msgRS4.set(RS.Position));
    request(RS_ID, V_PERCENTAGE);
    wait(2000, C_SET, V_PERCENTAGE);

  #endif

  #ifdef DIMMER
    send(msgIO.setSensor(DIMMER_ID).set(false));
    request(DIMMER_ID, V_STATUS);
    wait(2000, C_SET, V_STATUS);
    
    send(msgDIM.set(0));
    request(DIMMER_ID, V_PERCENTAGE);
    wait(2000, C_SET, V_PERCENTAGE);

  #endif

  #ifdef RGB
    send(msgIO.setSensor(DIMMER_ID).set(false));
    request(DIMMER_ID, V_STATUS);
    wait(2000, C_SET, V_STATUS);

    send(msgDIM.set(0));
    request(DIMMER_ID, V_PERCENTAGE);
    wait(2000, C_SET, V_PERCENTAGE);

    send(msgDIM2.set("000000"));
    request(DIMMER_ID, V_RGB);
    wait(2000, C_SET, V_RGB);

  #endif

  #ifdef RGBW
    send(msgIO.setSensor(DIMMER_ID).set(false));
    request(DIMMER_ID, V_STATUS);
    wait(2000, C_SET, V_STATUS);

    send(msgDIM.set(0));
    request(DIMMER_ID, V_PERCENTAGE);
    wait(2000, C_SET, V_PERCENTAGE);

    send(msgDIM3.set("00000000"));
    request(DIMMER_ID, V_RGBW);
    wait(2000, C_SET, V_RGBW);

  #endif

  #ifdef SPECIAL_BUTTON
    send(msgIO.setSensor(SPECIAL_BUTTON_ID).set(0));
  #endif

  // Built-in sensors
  #ifdef POWER_SENSOR
    send(msgPS.set("0"));
  #endif

  // External sensors
  #ifdef SHT30
    ETUpdate();
  #endif

  //
  #ifdef ELECTRONIC FUSE
    send(msgSI.setSensor(ES_ID).set(0));
  #endif

  #ifdef SHT30
    send(msgSI.setSensor(ETS_ID).set(0));
  #endif

  #ifdef RS485_DEBUG
    send(msgDEBUG.setSensor(DEBUG_ID).set("DEBUG MESSAGE"));
  #endif

  InitConfirm = true;

}


/*  *******************************************************************************************
                                        MySensors Receive
 *  *******************************************************************************************/
void receive(const MyMessage &message)  {
  
  if (message.type == V_STATUS) {
    #ifdef SPECIAL_BUTTON
      if (message.sensor == SPECIAL_BUTTON_ID)  {
        // Ignore this message
      }
    #endif
    #if defined(DIMMER) || defined(RGB) || defined(RGBW)
      if (message.sensor == DIMMER_ID) {
        Dimmer.NewState = message.getBool();
        Dimmer.ChangeState();
      }
    #endif
    #if defined(SINGLE_RELAY) || defined(DOUBLE_RELAY)
      if (message.sensor >= RELAY_ID_1 && message.sensor < NUMBER_OF_RELAYS)  {
        if (!OVERCURRENT_ERROR) {
          IO[message.sensor].NewState = message.getBool();
          IO[message.sensor].SetRelay();
        }
      }
    #endif
  }
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
        Dimmer.ChangeLevel();
      }
    #endif
  }
  else if (message.type == V_RGB || message.type == V_RGBW) {
    #if defined(RGB) || defined(RGBW)
      if(message.sensor == DIMMER_ID) {
        const char *rgbvalues = message.getString();

        Dimmer.NewState = true;
        Dimmer.NewColorValues(rgbvalues);
        Dimmer.ChangeColors();
      }
    #endif
  }
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
      send(msgETT.setDestination(0).set(sht.getTemperature(), 1));
      send(msgETH.set(sht.getHumidity(), 1));
      #ifdef HEATING_SECTION_SENSOR
        send(msgETT.setDestination(MY_HEATING_CONTROLLER).set(sht.getTemperature(), 1));
      #endif
    }
    else  {
      ET_ERROR = 1;
      send(msgSI.setSensor(ETS_ID).set(ET_ERROR));
    }
  #endif
}

/*  *******************************************************************************************
                                        Universal Input
 *  *******************************************************************************************/
void IOUpdate() {

  int FirstSensor = 0;
  int Iterations = NUMBER_OF_RELAYS+NUMBER_OF_INPUTS;

  if (Iterations > 0)  {
    for (int i = FirstSensor; i < FirstSensor + Iterations; i++)  {
      IO[i].ReadInput(TOUCH_THRESHOLD, LONGPRESS_DURATION, DEBOUNCE_VALUE);
      if (IO[i].NewState != IO[i].OldState)  {
        switch(IO[i].SensorType)  {
          case 0:
            // Touch Fields only (Hardware: RGBW)
            #ifdef DIMMER_ID
              if(i == 0)  {
                if(IO[i].NewState != 2) {
                  // Change dimmer status
                  Dimmer.NewState = !Dimmer.NewState;
                  send(msgIO.setSensor(DIMMER_ID).set(Dimmer.NewState));
                  Dimmer.ChangeState();
                  IO[i].OldState = IO[i].NewState;
                }
                #ifdef SPECIAL_BUTTON
                  if(IO[i].NewState == 2) {
                    send(msgIO.setSensor(SPECIAL_BUTTON_ID).set(true));
                    IO[i].NewState = IO[i].OldState;
                  }
                #endif  
              }
              else if(i == 1) {
                if(IO[i].NewState != 2)  {
                  if(Dimmer.NewState) {
                    // Toggle dimming level by DIMMING_TOGGLE_STEP
                    Dimmer.NewDimmingLevel += DIMMING_TOGGLE_STEP;

                    Dimmer.NewDimmingLevel = Dimmer.NewDimmingLevel > 100 ? DIMMING_TOGGLE_STEP : Dimmer.NewDimmingLevel;
                    send(msgDIM.set(Dimmer.NewDimmingLevel));
                    Dimmer.ChangeLevel();
                    IO[i].OldState = IO[i].NewState;
                  }
                }
              }
            #endif
          case 1:
            // Touch Fields & External buttons (Hardware: 2Relay)
            #ifdef ROLLER_SHUTTER
              if(IO[i].NewState != 2)  {
                MovementTime = RS.ReadButtons(i);
                IO[i].OldState = IO[i].NewState;
              }
              else  {
                #ifdef SPECIAL_BUTTON
                  send(msgIO.setSensor(SPECIAL_BUTTON_ID).set(true));
                  IO[i].NewState = IO[i].OldState;
                #endif
              }
            #else
              if (IO[i].NewState != 2)  {
                if (!OVERCURRENT_ERROR)  {
                  IO[i].SetRelay();
                  send(msgIO.setSensor(i).set(IO[i].NewState));
                }
              }
              #ifdef SPECIAL_BUTTON
                else if (IO[i].NewState == 2)  {
                  send(msgIO.setSensor(SPECIAL_BUTTON_ID).set(true));
                  IO[i].NewState = IO[i].OldState;
                }
              #endif
            #endif
            break;
          default:
            // Nothing to do here
          break;
        }
      }
    }
  }
}

/*  *******************************************************************************************
                                    Roller Shutter Calibration
 *  *******************************************************************************************/
void RSCalibration(float Vcc)  {

  #if defined(ROLLER_SHUTTER) && defined(RS_AUTO_CALIBRATION)

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

  RS.Calibration(UpTime, DownTime);

  #endif
    
}

/*  *******************************************************************************************
                                        Roller Shutter
 *  *******************************************************************************************/
void RSUpdate() {

  #ifdef ROLLER_SHUTTER

  uint32_t StopTime = 0;
  uint32_t MeasuredTime;
  bool Direction;

  if(RS.State != RS.NewState) {
    if(RS.NewState != 2)  {
      RS.Movement();
      StartTime = millis();
      if(RS.NewState == 0)  {
        send(msgRS1);
      }
      else if(RS.NewState == 1) {
        send(msgRS2);
      }
    }
    else  {
      Direction = RS.State;
      RS.Movement();
      StopTime = millis();
      send(msgRS3);
    }
  }

  if(RS.State != 2) {
    if(millis() >= StartTime + MovementTime) {
      Direction = RS.State;
      RS.NewState = 2;
      RS.Movement();
      StopTime = millis();
      send(msgRS3);
    }
    if(millis() < StartTime)  {
      uint32_t Temp = 4294967295 - StartTime + millis();
      wait(MovementTime - Temp);
      RS.NewState = 2;
      RS.Movement();
      send(msgRS3);
      StartTime = 0;
      StopTime = MovementTime;
    }
  }

  if(StopTime > 0)  {
    MeasuredTime = StopTime - StartTime;
    RS.CalculatePosition(Direction, MeasuredTime);
  
    send(msgRS4.set(RS.Position));
  }

  #endif
}

/*  *******************************************************************************************
                                        Power Sensor
 *  *******************************************************************************************/
void PSUpdate(float Current, uint8_t Sensor = 0)  {
  
  #if defined(POWER_SENSOR)
    send(msgPS.set(PS.CalculatePower(Current, COSFI), 0));
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
  if (NUMBER_OF_RELAYS + NUMBER_OF_INPUTS > 0) {
    IOUpdate();
  }

  // Updating roller shutter
  #ifdef ROLLER_SHUTTER
    RSUpdate();
  #endif

  // Reading power sensor(s)
  #ifdef POWER_SENSOR
    #if defined(SINGLE_RELAY) || defined(DOUBLE_RELAY) || defined(ROLLER_SHUTTER)
      if (digitalRead(RELAY_PIN_1) == RELAY_ON || digitalRead(RELAY_PIN_2) == RELAY_ON)  {
        Current = PS.MeasureAC(Vcc);
      }
    #elif defined(DIMMER) || defined(RGB) || defined(RGBW)
      if (Dimmer.NewState)  {
        Current = PS.MeasureDC(Vcc);
      }
    #endif
      
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
      #if defined(SINGLE_RELAY) || defined(DOUBLE_RELAY)
        for (int i = RELAY_ID_1; i < RELAY_ID_1 + NUMBER_OF_RELAYS; i++)  {
          IO[i].NewState = RELAY_OFF;
          IO[i].SetRelay();
          send(msgIO.setSensor(i).set(IO[i].NewState));
        }
      #elif defined(ROLLER_SHUTTER)
        RS.NewState = 2;
        RSUpdate();
      #elif defined(DIMMER) || defined(RGB) || defined(RGBW)
        Dimmer.NewState = false;
        Dimmer.ChangeState();
        send(msgIOD.setSensor(DIMMER_ID).set(Dimmer.NewState));
      #endif
      send(msgSI.setSensor(ES_ID).set(OVERCURRENT_ERROR));
      InformControllerES = true;
    }
    else if(!OVERCURRENT_ERROR && InformControllerES)  {
      // Current normal (only after reporting error)
      send(msgSI.setSensor(ES_ID).set(OVERCURRENT_ERROR));
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
    LastUpdate = millis();
    CheckNow = false;
  }

  wait(LOOP_TIME);
}
/*

   EOF

*/
