#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
    #include <avr/power.h>
#endif

#include "ardprintf.h"

// General Config
#define DEBUG                    false

// BLE Config
#define FACTORYRESET_ENABLE      0
#define BLE_READPACKET_TIMEOUT   20  // time in ms to wait for command from phone

// Neopixel Config
#define PIXELPIN        12   // Pin used to drive the NeoPixels
#define NUMPIXELS       1   // Number of neopixels in chain

// Setup drivers
Adafruit_BluefruitLE_UART ble(Serial1, -1);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN);

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

// declare functions & data defined in packetParser.CPP
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
extern uint8_t packetbuffer[];

// Log helping macros
#define log(msg) do { if(DEBUG) Serial.println(msg); } while (0)
#define logf(...) do { if(DEBUG) ardprintf(__VA_ARGS__); } while (0)
#define err(msg) do { if(DEBUG) Serial.println(msg); while (1); } while (0)

// State tracking
#define STATE_INITIALIZE 1
#define STATE_IDLE       2
#define STATE_COLOR_LOOP 3
int currentState = STATE_INITIALIZE;

int initializeCounter = 0;

uint32_t currentColor = 0;
uint32_t currentBrightness = 255;

int colorLoopCounter = 0;
#define colorLoopLength 4
uint32_t colorLoopColors[] = {
  pixels.Color(0, 0, 255),
  pixels.Color(0, 255, 255),
  pixels.Color(255, 255, 0),
  pixels.Color(255, 0, 0),
};

uint32_t prettyColor = pixels.Color(63, 240, 255);

void setup() {
    if (DEBUG) {
        // Wait for Serial connection
        while (!Serial);
        delay(500);

        Serial.begin(115200);
        log(F("Starting up MagicSkirt"));
    }

    // Initialize the NeoPixel library
    pixels.begin();
    pixels.show();

    // Initialize the 9DOF sensor
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

    /* Initialise the sensor */
    if(!lsm.begin())
    {
        /* There was a problem detecting the LSM9DS0 ... check your connections */
        err(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    }
    log(F("Found LSM9DS0 9DOF"));

    log(F("Initialising the Bluefruit LE module: "));
    if (!ble.begin(1)) {
        err(F("Couldn't find Bluefruit, make sure it's in command mode & check wiring?"));
    }
    log(F("Finished Initializing BLE module!"));

    // Factory Reset
    if (FACTORYRESET_ENABLE) {
        /* Perform a factory reset to make sure everything is in a known state */
        log(F("Performing a factory reset: "));
        if (!ble.factoryReset()) {
            err(F("Couldn't factory reset"));
        }
    }

    /* Disable command echo from Bluefruit */
    ble.echo(false);

    log("Requesting Bluefruit info:");
    /* Print Bluefruit information */
    ble.info();

    ble.verbose(false);  // debug info is a little annoying after this point!

    // Set module to DATA mode
    log(F("Switching to DATA mode!"));
    ble.setMode(BLUEFRUIT_MODE_DATA);
}

void loop() {
    if (currentState == STATE_INITIALIZE) {
        if (initializeCounter < 255) {
            initializeCounter += 10;
            currentColor = pixels.Color(0, 0, initializeCounter);
        } else {
            currentState = STATE_IDLE;
            currentColor = prettyColor;
        }
    } else if (currentState == STATE_IDLE) {
        uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

        // check to see if we got a color
        if (len > 0) {
          switch(packetbuffer[1]) {
            case 'C': {
              uint8_t red = packetbuffer[2];
              uint8_t green = packetbuffer[3];
              uint8_t blue = packetbuffer[4];

              logf("Setting pixel RGB(%d, %d, %d)", red, green, blue);
              currentColor = pixels.Color(red, green, blue);
              break;
            }
            case 'B': {
              uint8_t buttnum = packetbuffer[2] - '0';
              boolean pressed = packetbuffer[3] - '0';
              if (pressed) {
                if (buttnum == 1) {
                  currentState = STATE_COLOR_LOOP;
                } else if (buttnum == 2) {
                  currentColor = prettyColor;
                }
              }
              break;
            }
          }
        }
    } else if (currentState == STATE_COLOR_LOOP) {
      currentColor = colorLoopColors[colorLoopCounter % colorLoopLength];
      logf("Current Color: %d", currentColor);
      colorLoopCounter++;

      if (colorLoopCounter > 10) {
        colorLoopCounter = 0;
        currentState = STATE_IDLE;
      }
    }

    // Check our current movement vectors
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    // print out gyroscopic data
    float rot = abs(gyro.gyro.pitch);
    uint8_t brightness = int((rot / 180) * 255);
    if (brightness < 255 && brightness >= 0) {
        logf("Setting brightness: %d from rot: %f", brightness, rot);
        currentBrightness = brightness;
    }

    // Update any pixel changes
    pixels.setPixelColor(0, currentColor);
    pixels.setBrightness(currentBrightness);
    pixels.show();

    delay(100);
}
