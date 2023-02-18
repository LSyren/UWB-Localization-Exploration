/*
Makerfabs ESP32 UWB DW1000,
Anchor code, for communication with single tag.
*/

#include <SPI.h>
#include "DW1000Ranging.h"

#if defined(UWB_TAG)
#define ADDR_TAG "87:17:5B:D5:A9:9A:E2:9C"
#elif defined(UWB_ANCHOR)
#define ADDR_ANCHOR "87:17:5B:D5:A9:9A:E2:9C"
#endif

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
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
}

void newBlink(DW1000Device *device)
{
    Serial.print("blink; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
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
    SPI.begin(115200);
    #endif

    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

    //Set leds
    DW1000.enableDebounceClock();
    DW1000.enableLedBlinking();
    // enable RXOKLED
    DW1000.setGPIOMode(MSGP0, LED_MODE);
    // enable SFDLED
    DW1000.setGPIOMode(MSGP1, LED_MODE);
    // enable GPIO2/RXLED blinking
    DW1000.setGPIOMode(MSGP2, LED_MODE);
    // enable GPIO3/TXLED blinking
    DW1000.setGPIOMode(MSGP3, LED_MODE);

    // enable EXTTXE - TX antenna activity
    DW1000.setGPIOMode(MSGP5, LED_MODE);
    // enable EXTRXE - RX antenna activity
    DW1000.setGPIOMode(MSGP6, LED_MODE);

    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //Enable the filter to smooth the distance
    //DW1000Ranging.useRangeFilter(true);

    //we start the module as an anchor
    // DW1000Ranging.startAsAnchor("82:17:5B:D5:A9:9A:E2:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);

#if defined(UWB_ANCHOR)
    DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
#elif defined(UWB_TAG)
    DW1000Ranging.startAsTag(ADDR_TAG, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
#endif
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_LONGDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop()
{
    DW1000Ranging.loop();
}
