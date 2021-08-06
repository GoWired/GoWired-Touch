/*
 * Configuration.h file
 * Use this file for quick and easy configuration of your GetWired software.
 * For the whole description have a look at .ino file.
 * 
 */

#ifndef Configuration_h
#define Configuration_h

/*  *******************************************************************************************
                    Pinout
 *  *******************************************************************************************/
// Serial pin: 0 & 1
// EEPROM CS pin: 8
// SPI pins: 11, 12, 13

// Digital pins
#define DIGITAL_PIN_1 2
#define DIGITAL_PIN_2 3
#define DIGITAL_PIN_3 9
#define DIGITAL_PIN_4 10

// Analog pins
#define ANALOG_PIN_1 A0
#define ANALOG_PIN_2 A1
#define ANALOG_PIN_3 A2
#define ANALOG_PIN_4 A6
#define ANALOG_PIN_5 A7

// Dip switches
#define DIP_SWITCH_1 5
#define DIP_SWITCH_2 6
#define DIP_SWITCH_3 7

// Other
#define LP5009_ENABLE_PIN A3
#define RS485_ENABLE_PIN 4

// I2C
#define SDA A4
#define SCL A5
#define SDA1 23
#define SCL1 24

/*  *******************************************************************************************
                    MySensors Definitions
 *  *******************************************************************************************/
// Identification
#define MY_NODE_ID 1                            // Set node ID
#define SN "GetWired Module"               // Set node name to present to a controller
#define SV "1.0"                                // Set sensor version

// Selecting transmission settings
#define MY_RS485                              // Enable RS485 transport layer
#define MY_RS485_DE_PIN RS485_ENABLE_PIN      // DE Pin definition
#define MY_RS485_BAUD_RATE 57600              // Set RS485 baud rate
#define MY_RS485_HWSERIAL Serial              // Enable Hardware Serial
#define MY_RS485_SOH_COUNT 3                  // Collision avoidance

// FOTA Feature
#define MY_OTA_FIRMWARE_FEATURE                 // Enable OTA feature

// Other
#define MY_TRANSPORT_WAIT_READY_MS 60000        // Time to wait for gateway to respond at startup (default 60000)

/*  *******************************************************************************************
                    Quick config
 *  *******************************************************************************************/
// Output Config - one of these has to be defined
#define SINGLE_RELAY
//#define DOUBLE_RELAY              // Define this node as a double relay node, setting below
//#define ROLLER_SHUTTER            // Define this node as a roller shutter node, setting below
//#define DIMMER                    // Define this node as a 1-channel dimmer node, setting below
//#define RGB                       // Define this node as a RGB dimmer node, setting below
//#define RGBW                      // Define this node as a RGBW dimmer node, setting below

// Board dependent
#define POWER_SENSOR

// Onboard temperature sensor - SHT30
#define SHT30

/*  *******************************************************************************************
                    Various Definitions
 *  *******************************************************************************************/
// General
#define RELAY_ON HIGH                       // Pin state to turn the relays on (default HIGH)
#define RELAY_OFF LOW                       // Pin state to turn the relays off (default LOW)

// Hardware detection
#define HARDWARE_DETECTION_PIN ANALOG_PIN_1 // A6 or A7, tbd

// Reading inputs
#define TOUCH_THRESHOLD 5                   // A threshold to determine if it was a touch what was sensed (default 5, max. 65535)
#define DEBOUNCE_VALUE 20                   // Debounce time in ms (1 - no debounce, >1 active debounce, default 20, max. 255)
#define LONGPRESS_DURATION 1000             // Duration of longpress in ms (default 1000, max. 65535)

// Power Sensor
#define MAX_CURRENT 10                      // Maximum current the module can handle before reporting error (2SSR - 3; 4RelayDin - 10A or 16)
#define POWER_MEASURING_TIME 20             // Current measuring takes this long (default 20)
#define MVPERAMP 73.3                       // mV per 1A (default: 2SSR 185 mV/A; 4RelayDin 73.3 mV/A, RGBW 100 mV/A)
#define RECEIVER_VOLTAGE 230                // 230V, 24V, 12V - values for power usage calculation, depends on the receiver
#define COSFI 1                             // cos(fi) value for a given load: resistive load - 1, LED - 0.4 < cos(fi) < 0.99, fluorescent - 

// Dimmer
#define DIMMING_STEP 1                      // Size of dimming step, increase for faster, less smooth dimming (default 1)
#define DIMMING_INTERVAL 1                  // Duration of dimming interval, increase for slower dimming (default 10)
#define DIMMING_TOGGLE_STEP 20              // Value to increase dimming percentage when using wall switch

// Roller Shutter
#define RS_AUTO_CALIBRATION                 // Roller shutter auto-calibration. Leave it defined or comment it out and define movement times manually
#ifdef RS_AUTO_CALIBRATION
  #define PS_OFFSET 0.2                     // Power sensor offset for roller shutter calibration (default 0.2)
  #define CALIBRATION_SAMPLES 2             // Number of calibration samples for roller shutter calibration (default 2)
#else
  #define UP_TIME 21                        // Manually defined upward movement time in seconds (0-255)
  #define DOWN_TIME 20                      // Manually defined downward movement time in seconds (0-255)
#endif

// LP5009


// Heating system section thermometer
//#define HEATING_SECTION_SENSOR            // Define if this module if going to be a temperature sensor for a heating controller
#ifdef HEATING_SECTION_SENSOR
  #define MY_HEATING_CONTROLLER 1           // Node ID to which this module should report external temperature to
#endif

// Other
#define INTERVAL 300000                     // Interval value for reporting readings of the sensors: temperature, power usage (default 300000)
#define INIT_DELAY 200                      // A value to be multiplied by node ID value to obtain the time to wait during the initialization process
#define PRESENTATION_DELAY 10               // Time (ms) to wait between subsequent presentation messages (default 10)
#define LOOP_TIME 80                        // Main loop wait time (default 100)

/*  *******************************************************************************************
                    OUTPUT Config
 *  *******************************************************************************************/
#ifdef SINGLE_RELAY
  #define RELAY_ID_1 0
  #define RELAY_PIN_1 DIGITAL_PIN_3
  #define TOUCH_FIELD_1 ANALOG_PIN_3
  #define INPUT_PIN_1 DIGITAL_PIN_1
  #define NUMBER_IF_RELAYS 1
#endif

// 2SSR DOUBLE_RELAY
#ifdef DOUBLE_RELAY
  #define RELAY_ID_1 0
  #define RELAY_ID_2 1
  #define RELAY_PIN_1 DIGITAL_PIN_3
  #define RELAY_PIN_2 DIGITAL_PIN_4
  #define TOUCH_FIELD_1 ANALOG_PIN_1
  #define TOUCH_FIELD_2 ANALOG_PIN_2
  #define INPUT_PIN_1 DIGITAL_PIN_1
  #define INPUT_PIN_2 DIGITAL_PIN_2
  #define NUMBER_OF_RELAYS 2
#endif

// Roller Shutter
#ifdef ROLLER_SHUTTER
  #define RS_ID 0
  #define RELAY_ID_1 0
  #define RELAY_PIN_1 DIGITAL_PIN_3
  #define RELAY_PIN_2 DIGITAL_PIN_4
  #define INPUT_PIN_1 DIGITAL_PIN_1
  #define INPUT_PIN_2 DIGITAL_PIN_2
  #define NUMBER_OF_RELAYS 2
#endif

// Dimmer / RGB / RGBW
#ifdef DIMMER
  #define DIMMER_ID 0
  #define LED_PIN_W DIGITAL_PIN_4
  #define NUMBER_OF_CHANNELS 1
#endif

#ifdef RGB
  #define DIMMER_ID 0
  #define LED_PIN_R DIGITAL_PIN_1
  #define LED_PIN_G DIGITAL_PIN_2
  #define LED_PIN_B DIGITAL_PIN_3
  #define NUMBER_OF_CHANNELS 3
#endif

#ifdef RGBW
  #define DIMMER_ID 0
  #define LED_PIN_R DIGITAL_PIN_1
  #define LED_PIN_G DIGITAL_PIN_2
  #define LED_PIN_B DIGITAL_PIN_3
  #define LED_PIN_W DIGITAL_PIN_4
  #define NUMBER_OF_CHANNELS 4
#endif

#ifdef NUMBER_OF_RELAYS
  #define FIRST_INPUT_ID NUMBER_OF_RELAYS
#elif defined(NUMBER_OF_CHANNELS)
  #define FIRST_INPUT_ID 2
  #define NUMBER_OF_RELAYS 2
#else
  #define NUMBER_OF_RELAYS 0
  #define FIRST_INPUT_ID 0
#endif

/*  *******************************************************************************************
                    INPUT Config
 *  *******************************************************************************************/
#ifndef NUMBER_OF_INPUTS
  #define NUMBER_OF_INPUTS 0
#endif

// Special Button
#define SPECIAL_BUTTON
#define SPECIAL_BUTTON_ID FIRST_INPUT_ID

// ACS712 Power Sensor
#ifdef POWER_SENSOR
  #define PS_ID SPECIAL_BUTTON_ID+1
  #define PS_PIN ANALOG_PIN_2
#endif

// 1wire external thermometer (e.g. DHT22)
#ifdef EXTERNAL_TEMP
  #define ETT_ID PS_ID+1
  #define ETH_ID ETT_ID+1
#endif

/*  *******************************************************************************************
                      ERROR REPORTING & PREVENTION
 *  *******************************************************************************************/
#define ENABLE_WATCHDOG 

#define ELECTRONIC_FUSE
#ifdef ELECTRONIC_FUSE
  #define ES_ID ETH_ID+1
#endif

#ifdef SHT30
  #define ETS_ID ES_ID+1
#endif

//#define RS485_DEBUG
#ifdef RS485_DEBUG
  #define DEBUG_ID ETS_ID+1
#endif

/*  *******************************************************************************************
                    EEPROM Definitions
 *  *******************************************************************************************/
#define SIZE_OF_BYTE 1
#define EEPROM_OFFSET 512                               // First eeprom address to use (prior addresses are taken)

// Configuration parameters (version 1.0.0)
//#define ERASE_CONFIG_PIN
//#define HARDWARE_VERSION
//#define DETECTED_LOAD
//
// Saving States
//#define CHANNEL_1_STATE
//#define CHANNEL_2_STATE
//#define CHANNEL_3_STATE
//#define CHANNEL_4_STATE
//#define LED_INTENSITY
//#define DIMMER_STATE

// Roller Shutter
#define EEA_RS_TIME_DOWN EEPROM_OFFSET                  // EEPROM address to save RShutter travel down time
#define EEA_RS_TIME_UP EEA_RS_TIME_DOWN+SIZE_OF_INT     // EEPROM address to save RShutter travel up time
#define EEA_RS_POSITION EEA_RS_TIME_UP+SIZE_OF_INT      // EEPROM address to save RShutter last known position

#endif
/*
   EOF
*/
