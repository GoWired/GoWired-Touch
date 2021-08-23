/*
 Configuration
*/

/* Basic definitions */
// Version
#define GOWIRED
#define VERSION_DETECT_PIN A7               // Auto-detect connected hardware (A5 or A7)
#define NUMBER_OF_BUTTONS 2                 // Determined by hardware

// MySensors definitions
#ifdef GOWIRED
  // Identification
  #define MY_NODE_ID AUTO
  #define SN "GoWired Touch"               // Set node name to present to a controller
  #define SV "1.0"                         // Set sensor version

  // Transmission settings
  #define MY_RS485                              // Enable RS485 transport layer
  #define MY_RS485_DE_PIN 4                     // DE Pin definition (default 4)
  #define MY_RS485_BAUD_RATE 57600              // Set RS485 baud rate
  #define MY_RS485_HWSERIAL Serial              // Enable Hardware Serial
  #define MY_RS485_SOH_COUNT 3                  // Collision avoidance

  // FOTA Feature
  #define MY_OTA_FIRMWARE_FEATURE                 // Enable OTA feature

  // Other
  #define MY_TRANSPORT_WAIT_READY_MS 60000        // Time to wait for gateway to respond at startup (default 60000)
#endif

// Calibration
#define TOUCH_THRESHOLD 5                   // A threshold to determine if it was a touch what was sensed (default 5)

// LEDs
#define NUMBER_OF_CHANNELS 3                // RGB LEDs
#define DIMMING_STEP 1                      // Size of dimming step, increase for faster, less smooth dimming (default 1)
#define DIMMING_INTERVAL 1                  // Duration of dimming interval, increase for slower dimming (default 10)
#define R_RELAY_OFF 0
#define G_RELAY_OFF 0
#define B_RELAY_OFF 255
#define R_RELAY_ON 255
#define G_RELAY_ON 50
#define B_RELAY_ON 0
#define LEVEL_RELAY_OFF 20
#define LEVEL_RELAY_ON 40
#define LEVEL_RELAY_OFF_2 40
#define LEVEL_RELAY_ON_2 80
#define CALIBRATION_SIGNALS 3

// Relay states
#define RELAY_ON HIGH
#define RELAY_OFF LOW

// Roller Shutter
#define RS_INTERVAL 80                 // Time in seconds to turn off roller shutter after it was turned on (deafult 80)

/* Pin Definitions */
// Touch buttons 
#define BUTTON_PIN_1 A0
#define BUTTON_PIN_2 A1
#define BUTTON_PIN_3 A2

// AC inputs
/*#define INPUT_PIN_1 4
#define INPUT_PIN_2 7

// Relays
#define RELAY_PIN_1 8
#define RELAY_PIN_2 23*/

// LED pins
#define LED_PIN_1 1
#define LED_PIN_2 2
#define LED_PIN_3 3

#define LED_PIN_4 0
#define LED_PIN_5 5
#define LED_PIN_6 6

#define LED_PIN_7 9
#define LED_PIN_8 10
#define LED_PIN_9 11

// DIP switch
#define SETUP_PIN_1 A3
#define SETUP_PIN_2 A4
#define SETUP_PIN_3 A6
#define SETUP_PIN_4 A7

/* Reliability */
// Watchdog
#define ENABLE_WATCHDOG

/* EEPROM Addresses */
#define SIZE_OF_BYTE 1
#define EEPROM_OFFSET 512                         // First eeprom address to use (prior addresses are taken)
#define EEA_RELAY_1 EEPROM_OFFSET+SIZE_OF_BYTE            // EEPROM addresses to save relay states
#define EEA_RELAY_2 EEA_RELAY_1+SIZE_OF_BYTE
#define EEA_RS_TIME_DOWN EEA_RELAY_2+SIZE_OF_BYTE            // EEPROM address to save RShutter travel down time
#define EEA_RS_TIME_UP EEA_RS_TIME_DOWN+SIZE_OF_BYTE     // EEPROM address to save RShutter travel up time
#define EEA_RS_POSITION EEA_RS_TIME_UP+SIZE_OF_BYTE      // EEPROM address to save RShutter last known position

