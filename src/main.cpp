/*
Makerfabs ESP32 UWB DW1000,
Code for tag.
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

#define DETECTION_PIN 5

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

#endif

#ifdef BLUEPILL
#define DETECTION_PIN PA5

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

    if ((DW1000Ranging.getDistantDevice()->getRange() < 2.0))
    {
        digitalWrite(DETECTION_PIN, LOW);
        Serial.println("WITHIN");
    }
    else
    {
        digitalWrite(DETECTION_PIN, HIGH);
        Serial.println("OUTSIDE");
    }
}

void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
    digitalWrite(DETECTION_PIN, HIGH);
    Serial.println("LOW, lost device");
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

    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    //define the sketch as anchor. It will be great to dynamically change the type of module

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

    DW1000.setGPIOMode(MSGP5, LED_MODE);
    DW1000.setGPIOMode(MSGP6, LED_MODE);

    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //Enable the filter to smooth the distance
    //DW1000Ranging.useRangeFilter(true);

    //we start the module as a tag
    DW1000Ranging.startAsTag("7C:00:22:EA:82:60:3B:9C", DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    //DW1000.enableManualLedBlinking();

#if defined(UWB_ANCHOR)
    DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
#elif defined(UWB_TAG)
    DW1000Ranging.startAsTag(ADDR_TAG, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
#endif
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_LONGDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
    pinMode(DETECTION_PIN, OUTPUT);
}
void loop()
{
    DW1000Ranging.loop();
}