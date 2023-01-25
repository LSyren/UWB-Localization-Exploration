/*
Makerfabs ESP32 UWB DW1000,
Anchor code, for communication with tag/tags.
*/

#include <SPI.h>
#include "DW1000Ranging.h"

#define ANCHOR_ADD "86:17:5B:D5:A9:9A:E2:9C"

#define ANTENNA_DELAY 16514

#ifdef MAKERFABS

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

#endif

#ifdef BLUEPILL

#define SPI_SCK PB13
#define SPI_MISO PB14
#define SPI_MOSI PB15

// connection pins
const uint8_t PIN_RST = PA12; // reset pin
const uint8_t PIN_IRQ = PA11; // irq pin
const uint8_t PIN_SS = PB12;   // spi select pin

#endif

void newRange()
{
    if (abs(DW1000Ranging.getDistantDevice()->getRange()) > 0.3) 
    {
        Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
        Serial.print(":");
        Serial.print(DW1000Ranging.getDistantDevice()->getRange());

        Serial.flush();
    }
    /*
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
    */
}

void newBlink(DW1000Device *device)
{
    //Serial.print("blink; 1 device added ! -> ");
    //Serial.print(" short:");
    //Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    //Serial.print("delete inactive device: ");
    //Serial.println(device->getShortAddress(), HEX);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    //init the configuration
    #ifdef MAKERFABS
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    #endif

    #ifdef BLUEPILL
    SPI.setMISO(SPI_MISO);
	SPI.setMOSI(SPI_MOSI);
	SPI.setSCLK(SPI_SCK);
    SPI.begin(115200);
    #endif
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

    //Set leds
    DW1000.enableDebounceClock();
    DW1000.enableLedBlinking();
    // enable SFDLED
    DW1000.setGPIOMode(MSGP1, LED_MODE);
    // enable GPIO2/RXLED blinking
    DW1000.setGPIOMode(MSGP2, LED_MODE);
    // enable GPIO3/TXLED blinking
    DW1000.setGPIOMode(MSGP3, LED_MODE);

    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //Enable the filter to smooth the distance
    //DW1000Ranging.useRangeFilter(true);

    //we start the module as an anchor
    // DW1000Ranging.startAsAnchor("82:17:5B:D5:A9:9A:E2:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);

    DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop()
{
    DW1000Ranging.loop();
}
