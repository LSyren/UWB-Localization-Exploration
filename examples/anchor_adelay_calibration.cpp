/*
Makerfabs ESP32 UWB DW1000,
Anchor code, for communication with single tag.
*/

#include <SPI.h>
#include "DW1000Ranging.h"
#include <math.h>
#define ANCHOR_ADD "87:17:5B:D5:A9:9A:E2:9C"
float this_anchor_target_distance = 7.94; //measured distance to anchor in m

uint16_t this_anchor_Adelay = 16600; //starting value
uint16_t Adelay_delta = 100; //initial binary search step size

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
    static float last_delta = 0.0;
    Serial.println(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
    /*
    if (abs(DW1000Ranging.getDistantDevice()->getRXPower() + 104.0) < 1.0)
    {
        Serial.print("Distance for calibration: \t");
        Serial.println(Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC));
    }
    */
    float dist = 0;
    for (int i = 0; i < 100; i++)
    {
        // get and average 100 measurements
        dist += DW1000Ranging.getDistantDevice()->getRange();
    }
    dist /= 100.0;
    Serial.print(",");
    Serial.print(dist);
    if (Adelay_delta < 3)
    {
        Serial.print(", final Adelay ");
        Serial.println(this_anchor_Adelay);
        //    Serial.print("Check: stored Adelay = ");
        //    Serial.println(DW1000.getAntennaDelay());
        while (1)
            ; // done
    }

    float this_delta = dist - this_anchor_target_distance; // error in measured distance

    if (this_delta * last_delta < 0.0)
        Adelay_delta = Adelay_delta / 2; // sign changed, reduce step size
    last_delta = this_delta;

    if (this_delta > 0.0)
        this_anchor_Adelay += Adelay_delta; // new trial Adelay
    else
        this_anchor_Adelay -= Adelay_delta;

    Serial.print(", Adelay = ");
    Serial.println(this_anchor_Adelay);
    //  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    DW1000.setAntennaDelay(this_anchor_Adelay);
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
