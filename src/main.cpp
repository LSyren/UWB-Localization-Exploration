/*
Makerfabs ESP32 UWB DW1000,
Anchor and Tag.
*/

#include <SPI.h>
#include "DW1000Ranging.h"

#define RANGE_THRESHOLD_METERS 2.0f

const uint16_t this_anchor_Adelay = 16566; //starting value

#if defined(UWB_TAG)
#define ADDR_TAG "7F:00:22:EA:82:60:3B:9C"
#elif defined(UWB_ANCHOR)
#define ADDR_ANCHOR "87:17:5B:D5:A9:9A:E2:9C"
#endif

#if defined(MAKERFABS)

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

#define DETECTION_PIN 5

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

#elif defined(BLUEPILL)
#define DETECTION_PIN PA5

#define SPI_SCK PB13
#define SPI_MISO PB14
#define SPI_MOSI PB15

// connection pins
const uint8_t PIN_RST = PA12; // reset pin
const uint8_t PIN_IRQ = PA11; // irq pin
const uint8_t PIN_SS = PB12;   // spi select pin

#endif

#if defined(UWB_ANCHOR_AD)

uint16_t Adelay_delta = 100; //initial binary search step size
float this_anchor_target_distance = 7.0; //measured distance to anchor in m

void newRange()
{
  static float last_delta = 0.0;
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);

  float dist = 0;
  for (int i = 0; i < 100; i++) {
    // get and average 100 measurements
    dist += DW1000Ranging.getDistantDevice()->getRange();
  }
  dist /= 100.0;
  Serial.print(",");
  Serial.print(dist); 
  if (Adelay_delta < 3) {
    Serial.print(", final Adelay ");
    Serial.println(this_anchor_Adelay);
    while(1);  //done calibrating
  }

  float this_delta = dist - this_anchor_target_distance;  //error in measured distance

  if ( this_delta * last_delta < 0.0) Adelay_delta = Adelay_delta / 2; //sign changed, reduce step size
    last_delta = this_delta;
  
  if (this_delta > 0.0 ) this_anchor_Adelay += Adelay_delta; //new trial Adelay
  else this_anchor_Adelay -= Adelay_delta;
  
  Serial.print(", Adelay = ");
  Serial.println (this_anchor_Adelay);
//  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000.setAntennaDelay(this_anchor_Adelay);
}
#else
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
    #if defined(UWB_TAG)
    if ((DW1000Ranging.getDistantDevice()->getRange() < RANGE_THRESHOLD_METERS))
    {
        digitalWrite(DETECTION_PIN, LOW);
        Serial.println("WITHIN");
    }
    else
    {
        digitalWrite(DETECTION_PIN, HIGH);
        Serial.println("OUTSIDE");
    }
    #endif
}
#endif

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
    #if defined(UWB_TAG)
    digitalWrite(DETECTION_PIN, HIGH);
    #endif
    Serial.println("LOW, lost device");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    //init the configuration
    #if defined(MAKERFABS)
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    #elif defined(BLUEPILL)
    SPI.setMISO(SPI_MISO);
    SPI.setMOSI(SPI_MOSI);
    SPI.setSCLK(SPI_SCK);
    SPI.begin(115200);
    #endif

    #if defined(UWB_ANCHOR_AD)
    Serial.print("Starting Adelay "); Serial.println(this_anchor_Adelay);
    Serial.print("Measured distance "); Serial.println(this_anchor_target_distance);
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
    //DW1000.setGPIOMode(MSGP5, LED_MODE);
    // enable EXTRXE - RX antenna activity
    //DW1000.setGPIOMode(MSGP6, LED_MODE);

    DW1000.setGPIOMode(MSGP5, LED_MODE);
    DW1000.setGPIOMode(MSGP6, LED_MODE);

    DW1000.setGPIOMode(MSGP5, LED_MODE);
    DW1000.setGPIOMode(MSGP6, LED_MODE);

    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //Enable the filter to smooth the distance
    DW1000Ranging.useRangeFilter(false);

#if defined(UWB_ANCHOR)
    DW1000.setAntennaDelay(this_anchor_Adelay);
    DW1000Ranging.startAsAnchor(ADDR_ANCHOR, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
#elif defined(UWB_TAG)
    DW1000Ranging.startAsTag(ADDR_TAG, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    pinMode(DETECTION_PIN, OUTPUT);
    delay(1000);
    digitalWrite(DETECTION_PIN, HIGH);
#endif
}
void loop()
{
    DW1000Ranging.loop();
}
