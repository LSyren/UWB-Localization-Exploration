/*
Makerfabs ESP32 UWB DW1000,
Anchor and Tag.
*/

#include <SPI.h>
#include "DW1000Ranging.h"
#include "Smoothed.h"

#define RANGE_THRESHOLD_METERS 4.0f
uint16_t this_anchor_Adelay = 16514; //starting value 16566, 16539

#if defined(UWB_TAG)
#define ADDR_TAG "7B:00:22:EA:82:60:3B:9C"
#elif defined(UWB_ANCHOR)
#define ADDR_ANCHOR "04:17:5B:D5:A9:9A:E2:9C"
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

// variables for position determination
#define N_ANCHORS 4
#define ANCHOR_DISTANCE_EXPIRED 5000   //measurements older than this are ignore (milliseconds)

float anchor_matrix[N_ANCHORS][3] = { //list of anchor coordinates, relative to chosen origin.
    {0.0, 0.0, 0.97},  //Anchor labeled #1
    {0.0, 10.0, 1.14},//Anchor labeled #2
    {5.0, 10.0, 0.6}, //Anchor labeled #3
    { 5.0, 0.0, 0.15} //Anchor labeled #4
};  //Z values are ignored in this code, except to compute RMS distance error

Smoothed <float> xVals; 
Smoothed <float> yVals;

float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports
uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen

float current_tag_position[2] = {0.0, 0.0}; //global current position (meters with respect to anchor origin)
float current_distance_rmse = 0.0f;  //rms error in distance calc => crude measure of position error (meters).  Needs to be better characterized

int trilat2D_4A(void) {
    // for method see technical paper at
    // https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf
    // S. James Remington 1/2022
    //
    // A nice feature of this method is that the normal matrix depends only on the anchor arrangement
    // and needs to be inverted only once. Hence, the position calculation should be robust.
    //
    static bool first = true;  //first time through, some preliminary work
    float d[N_ANCHORS]; //temp vector, distances from anchors

    static float A[N_ANCHORS - 1][2];
    static float Ainv[2][2];
    static float b[N_ANCHORS - 1];
    static float kv[N_ANCHORS]; //calculated in first call, used in later calls

    int i, j, k;
    // copy distances to local storage
    for (i = 0; i < N_ANCHORS; i++) d[i] = last_anchor_distance[i];

#ifdef DEBUG_TRILAT
    char line[60];
    snprintf(line, sizeof line, "d: %6.2f %6.2f %6.2f", d[0], d[1], d[2]);
    Serial.println(line);
#endif

    if (first) {  //intermediate fixed vectors
        first = false;

        float x[N_ANCHORS], y[N_ANCHORS]; //intermediate vectors

        for (i = 0; i < N_ANCHORS; i++) {
            x[i] = anchor_matrix[i][0];
            y[i] = anchor_matrix[i][1];
            kv[i] = x[i] * x[i] + y[i] * y[i];
        }

        // set up least squares equation

        for (i = 1; i < N_ANCHORS; i++) {
            A[i - 1][0] = x[i] - x[0];
            A[i - 1][1] = y[i] - y[0];
#ifdef DEBUG_TRILAT
            snprintf(line, sizeof line, "A  %5.2f %5.2f", A[i - 1][0], A[i - 1][1]);
            Serial.println(line);
#endif
        }
        float ATA[2][2];  //calculate A transpose A
        // Cij = sum(k) (Aki*Akj)
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 2; j++) {
                ATA[i][j] = 0.0;
                for (k = 0; k < N_ANCHORS - 1; k++) ATA[i][j] += A[k][i] * A[k][j];
            }
        }
#ifdef DEBUG_TRILAT
        snprintf(line, sizeof line, "ATA %5.2f %5.2f\n    %5.2f %5.2f", ATA[0][0], ATA[0][1], ATA[1][0], ATA[1][1]);
        Serial.println(line);
#endif

        //invert ATA
        float det = ATA[0][0] * ATA[1][1] - ATA[1][0] * ATA[0][1];
        if (fabs(det) < 1.0E-4) {
            Serial.println("***Singular matrix, check anchor coordinates***");
            while (1) delay(1); //hang
        }
        det = 1.0 / det;
        //scale adjoint
        Ainv[0][0] =  det * ATA[1][1];
        Ainv[0][1] = -det * ATA[0][1];
        Ainv[1][0] = -det * ATA[1][0];
        Ainv[1][1] =  det * ATA[0][0];
#ifdef DEBUG_TRILAT
        snprintf(line, sizeof line, "Ainv %7.4f %7.4f\n     %7.4f %7.4f", Ainv[0][0], Ainv[0][1], Ainv[1][0], Ainv[1][1]);
        Serial.println(line);
        snprintf(line, sizeof line, "det Ainv %8.3e", det);
        Serial.println(line);
#endif

    } //end if (first);

    //least squares solution for position
    //solve:  (x,y) = 0.5*(Ainv AT b)

    for (i = 1; i < N_ANCHORS; i++) {
        b[i - 1] = d[0] * d[0] - d[i] * d[i] + kv[i] - kv[0];
    }

    float ATb[2] = {0.0}; //A transpose b
    for (i = 0; i < N_ANCHORS - 1; i++) {
        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }

    current_tag_position[0] = 0.5 * (Ainv[0][0] * ATb[0] + Ainv[0][1] * ATb[1]);
    current_tag_position[1] = 0.5 * (Ainv[1][0] * ATb[0] + Ainv[1][1] * ATb[1]);

    // calculate rms error for distances
    float rmse = 0.0, dc0 = 0.0, dc1 = 0.0, dc2 = 0.0;
    for (i = 0; i < N_ANCHORS; i++) {
        dc0 = current_tag_position[0] - anchor_matrix[i][0];
        dc1 = current_tag_position[1] - anchor_matrix[i][1];
        dc2 = anchor_matrix[i][2]; //include known Z coordinate of anchor
        dc0 = d[i] - sqrt(dc0 * dc0 + dc1 * dc1 + dc2 * dc2);
        rmse += dc0 * dc0;
    }
    current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS));

    return 1;
}  //end trilat2D_3A

int trilat2D_3A(void) {

  // for method see technical paper at
  // https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf
  // S. James Remington 1/2022
  //
  // A nice feature of this method is that the normal matrix depends only on the anchor arrangement 
  // and needs to be inverted only once. Hence, the position calculation should be robust.
  //
  static bool first = true;  //first time through, some preliminary work
  float b[N_ANCHORS], d[N_ANCHORS]; //temp vector, distances from anchors

  static float Ainv[2][2], k[N_ANCHORS]; //these are calculated only once

  int i;
  // copy distances to local storage
  for (i = 0; i < N_ANCHORS; i++) d[i] = last_anchor_distance[i];

#ifdef DEBUG_TRILAT
  char line[60];
  snprintf(line, sizeof line, "d: %6.2f %6.2f %6.2f", d[0], d[1], d[2]);
  Serial.println(line);
#endif

  if (first) {  //intermediate fixed vectors
    first = false;

    float x[N_ANCHORS], y[N_ANCHORS]; //intermediate vectors
    float A[2][2];  //the A matrix for system of equations to solve

    for (i = 0; i < N_ANCHORS; i++) {
      x[i] = anchor_matrix[i][0];
      y[i] = anchor_matrix[i][1];
      k[i] = x[i] * x[i] + y[i] * y[i];
    }

    // set up least squares equation

    for (i = 1; i < N_ANCHORS; i++) {
      A[i - 1][0] = x[i] - x[0];
      A[i - 1][1] = y[i] - y[0];
#ifdef DEBUG_TRILAT
      snprintf(line, sizeof line, "A  %5.2f %5.2f \n", A[i - 1][0], A[i - 1][1]);
      Serial.println(line);
#endif
    }
    //invert A
    float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (fabs(det) < 1.0E-4) {
      Serial.println("***Singular matrix, check anchor coordinates***");
      while (1) delay(1); //hang
    }

#ifdef DEBUG_TRILAT
    snprintf(line, sizeof line, "det A %8.3e\n", det);
    Serial.println(line);
#endif

    det = 1.0 / det;
    //scale adjoint
    Ainv[0][0] =  det * A[1][1];
    Ainv[0][1] = -det * A[0][1];
    Ainv[1][0] = -det * A[1][0];
    Ainv[1][1] =  det * A[0][0];
  } //end if (first);

  for (i = 1; i < N_ANCHORS; i++) {
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];
  }

  //least squares solution for position
  //solve:  2 A rc = b

  current_tag_position[0] = 0.5 * (Ainv[0][0] * b[0] + Ainv[0][1] * b[1]);
  current_tag_position[1] = 0.5 * (Ainv[1][0] * b[0] + Ainv[1][1] * b[1]);

  // calculate rms error for distances
  float rmse = 0.0, dc0 = 0.0, dc1 = 0.0;
  for (i = 0; i < N_ANCHORS; i++) {
    dc0 = current_tag_position[0] - anchor_matrix[i][0];
    dc1 = current_tag_position[1] - anchor_matrix[i][1];
    dc0 = d[i] - sqrt(dc0 * dc0 + dc1 * dc1);
    rmse += dc0 * dc0;
  }
  current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS));

  return 1;
}  //end trilat2D_3A

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
    /*
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
    */

    #if defined(UWB_TAG)
    /**
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
    */
    #endif

    // Update anchor distance.
    int anchor_idx = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x07;

    if (anchor_idx > 0 && anchor_idx <= N_ANCHORS) {
        last_anchor_update[anchor_idx - 1] = millis();
        float range = DW1000Ranging.getDistantDevice()->getRange();
        last_anchor_distance[anchor_idx - 1] = range;
        if (range < 0.0f || range > 30.0f) {
            last_anchor_update[anchor_idx - 1] = 0;
        }
    }

    int detected = 0;

    // Reject old measurement.
    for (int i = 0; i < N_ANCHORS; i++) {
        if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) {
            last_anchor_update[i] = 0;
        }
        if (last_anchor_update[i] > 0) {
	        detected++;
        }
    }

#ifdef DEBUG_DIST
    // print distance and age of measurement
    uint32_t current_time = millis();
    for (i = 0; i < N_ANCHORS; i++) {
        Serial.print(i+1); //ID
        Serial.print("> ");
        Serial.print(last_anchor_distance[i]);
        Serial.print("\t");
        Serial.println(current_time - last_anchor_update[i]); //age in millis
    }
#endif

    if (detected == 3) {
        trilat2D_3A();

        xVals.add(current_tag_position[0]);
        yVals.add(current_tag_position[1]);
        if (xVals.get() > -1.0 && xVals.get() < 10.0 && yVals.get() > -1.0 && yVals.get() < 10.0) {
            digitalWrite(DETECTION_PIN, LOW);
            //Serial.println("WITHIN");
        } else {
            digitalWrite(DETECTION_PIN, HIGH);
            //Serial.println("OUTSIDE");
        }

        //Serial.print("P= ");
        Serial.print(xVals.get());
        Serial.write(',');
        Serial.print(yVals.get());
        Serial.write(',');
        Serial.println(0.0);
        //Serial.println(current_distance_rmse);
    }
}
#endif

void newDevice(DW1000Device *device)
{
    //Serial.print("ranging init; 1 device added ! -> ");
    //Serial.print(" short:");
    //Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    //Serial.print("delete inactive device: ");
    //Serial.println(device->getShortAddress(), HEX);
    #if defined(UWB_TAG)
    digitalWrite(DETECTION_PIN, HIGH);
    #endif
    //Serial.println("LOW, lost device");
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

    // Initialise the first sensor value store. We want this to be the simple average of the last 10 values.
	// Note: The more values you store, the more memory will be used.
	xVals.begin(SMOOTHED_AVERAGE, 10);
    yVals.begin(SMOOTHED_AVERAGE, 10);	

	// Initialise the second sensor value store. We want this one to be a simple linear recursive exponential filter. 
	// We set the filter level to 10. Higher numbers will result in less filtering/smoothing. Lower number result in more filtering/smoothing
	//mySensor2.begin(SMOOTHED_EXPONENTIAL, 10);
#endif
}
void loop()
{
    DW1000Ranging.loop();
}
