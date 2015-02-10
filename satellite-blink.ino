/*

PA6H Satellite Blinker for Arduino
MIT License

Copyright (c) 2015 Patrick Tudor

This is the software for an ATTiny85 on a PCB I make that is a replacement
for the default GPS puck from Byonics for the MicroTrak RTG and similar.

The code does two things:

1) Connect to the GPS at 9600 default and drop it to 4800 before
   updating the list of NMEA sentences to output, and

2) Parse the NMEA stream and once a minute or so blink an LED with
   a count of the satellites in view so you know the GPS is working.

The PCB works like this:

1) DB9 with In, Out, 5V, and ground
2) In and Out to ST3232, 5V to RT9193-3.3
3) ST3232 to PA6H
4) PA6H to RF and ATTiny85
5) 1220 or 2032 coin cell for PA6H

The ATTiny85 should be configured to use its internal 8MHz clock
and TinyTuner3 should be used to calibrate it in advance if possible.

Enjoy,

*/

//
// ATTiny85
// Where ISP is red-orange-yellow-green-blue-violet
// 1: RST blue
// 2: three
// 3: four
// 4: GND violet
// 5: zero / MOSI green
// 6: one / MISO red
// 7: two / SCK yellow
// 8: VCC orange

// ATMEL ATTINY45 / ARDUINO
//
//                           +-\/-+
//  Ain0       (D  5)  PB5  1|    |8   VCC
//  Ain3       (D  3)  PB3  2|    |7   PB2  (D  2)  INT0  Ain1
//  Ain2       (D  4)  PB4  3|    |6   PB1  (D  1)        pwm1
//                     GND  4|    |5   PB0  (D  0)        pwm0
//                           +----+

#define PABY_VERSION "PA6H Monitor v1.0"
#define PABY_COPYRIGHT "(c) Patrick Tudor"

// this is the loop interval in milliseconds. LED will blink approximately this often.
// unsigned int so please don't exceed around 60,000, or change to long below.
#define INTERVAL 16000
// if the Oscillator has been calibrated, this pulls OSCCAL from EEPROM
#define ATTINY_CALIBRATED 1
// if using the MKT3339/PA6H chipset, set to one.
#define GPS_PA6H 1
// if parsing sentences to check for antenna, set to one. Please don't do that yet.
#define ANTENNA 0
// debug mode pushes simple data out the transmit pin
#define DEBUG 0
// PCB revision used to set the pins for the ATTiny85
#define PCB_A 1

// include files
#ifndef _HEADERS_PABY
#define _HEADERS_PABY
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <avr/wdt.h> // watchdog timer
#endif

// make a gps object
TinyGPSPlus gps;
// "There is an output sentence that will tell you the status of the antenna. $PGTOP,11,x where x is the status number.
//  If x is 3 that means it is using the external antenna. If x is 2 it's using the internal antenna and
//  if x is 1 there was an antenna short or problem."
#define PA6H_ANTENNA_STATUS "$PGCMD,33,1*6C"
#if ANTENNA
TinyGPSCustom antenna(gps, "PGTOP", 2); // $PGTOP sentence, 2nd element
#endif

// http://www.hhhh.org/wiml/proj/nmeaxor.html
// " The checksum is simple, just an XOR of all the bytes between the $ and the * (not including the delimiters themselves), and written in hexadecimal."
#define PA6H_MESSAGES_DEFAULT "$PMTK314,-1*04"
#define PA6H_MESSAGES "$PMTK314,3,1,3,1,2,6,0,0,0,0,0,0,0,0,0,0,0,0,0*2C"
#define PA6H_BAUD_4800 "$PMTK251,4800*14"
#define PA6H_BAUD_9600 "$PMTK251,9600*17"

//static const uint16_t GPS96 = 9600;
//static const uint16_t GPS48 = 4800;
//static const uint32_t GPSBaud = 4800;

#if PCB_A
// The serial connection to the GPS device
static const byte RXPin = 4, TXPin = 3;
SoftwareSerial ss(RXPin, TXPin);
// the LED pin
static const byte statusLED = 2;
#endif

// a counter for void loop(). Uses int instead of long to save 2 bytes.
unsigned int loop_counter;

// take a count, like 4, and a millsecond time, like 1000, and blink the LED.
void blinkDelay(byte count, int timecount) {
  for (int x = 0 ; x < count ; x++)
  {
    // reset the watchdog timer throughout just in case these are Long Delays.
    wdt_reset();
    // LED on,
    digitalWrite(statusLED, HIGH);
    delay(timecount);
    wdt_reset();
    // LED off,
    digitalWrite(statusLED, LOW);
    delay(timecount);
    wdt_reset();
  }
  //digitalWrite(statusLED, LOW);
}

void setup() {
  // eight second watchdog timer
  wdt_enable(WDTO_8S);

  // Use TinyTuner "Save_to_EEPROM" first to tune Oscillator
#if ATTINY_CALIBRATED
  OSCCAL =  EEPROM.read(0), HEX;
#endif

  // set pin modes
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, LOW);
  pinMode(RXPin, INPUT);
  pinMode(TXPin, OUTPUT);
  // give a "power on" blink, six times with 50ms delay
  blinkDelay(6, 50);

  // begin serial at 9600 default to set 4800
  ss.begin(9600);
#if GPS_PA6H
  ss.println(PA6H_BAUD_4800);
#endif
  ss.end();

  // begin serial at 4800
  ss.begin(4800);

  // push custom message interval list
#if GPS_PA6H
  //ss.println(PA6H_MESSAGES_DEFAULT);
  ss.println(PA6H_MESSAGES);
#endif

  // uses too much RAM. Don't enable.
#if ANTENNA
  ss.println(PA6H_ANTENNA_STATUS);  //softwareserial
#endif

  // print the OSCCAL value. Probably around 80.
#if DEBUG
  ss.print(F("OSCCAL:"));
  ss.println(EEPROM.read( 0 ), HEX);
#endif

  // give the first run a headstart
  loop_counter = INTERVAL - 4000;

}

void loop() {

  // read NMEA into gps object
  gps.encode(ss.read());

  // increment counter toward INTERVAL
  loop_counter++;
  // slow it down just a little tiny bit
  delay(1);

  // when we cross the threshold, update LED status
  if (loop_counter > INTERVAL)  {
    // first let's reset the counter
    loop_counter = 0;
    // notify user an update is arriving with a quick quarter-second blink
    blinkDelay(4, 60);
    // and pause for a second
    delay(1000);

    // let's make sure first that we have fresh data
    uint32_t fixage;
    fixage = gps.satellites.age();

    // average is in the range 1000-3000. 10 seconds is too long. "no data" is -1 unsigned or 4B.
    if (fixage > 10000) {
#if DEBUG
      ss.print(F("age:"));
      ss.println(fixage);
#endif
      // no data, solid LED for four seconds.
      blinkDelay(1, 4000);
      // probably temporary, try again in five seconds.
      loop_counter = INTERVAL - 5000;
    } else {
      // good data, let's count the satellites.
      int satellitesInView;
      satellitesInView = gps.satellites.value();
#if DEBUG
      ss.print(gps.charsProcessed());
      ss.print(F(","));
      ss.print(fixage);
      ss.print(F(","));
      ss.println(satellitesInView);
#endif
      // true even with zero satellites, as in, true with NMEA sentences coming in
      if (gps.location.isValid()) {

        // if there are zero or one satellites, two second blink.
        if ( satellitesInView <= 1 ) {

#if DEBUG
          ss.println(F("s0|1"));
#endif
          blinkDelay(1, 2000);

          // normal condition. Blink on/off for each satellite in view
        } else if ( satellitesInView > 1 ) {
#if DEBUG
          ss.println(F("s > 1"));
#endif
          blinkDelay(satellitesInView, 500);

          // this is an unexpected problem. Blink four times, two seconds each.
        } else {
#if DEBUG
          ss.println(F("!s"));
#endif
          blinkDelay(4, 2000);

        }

        // antenna check. useful but causes RAM problems so don't use it
#if ANTENNA
        byte antennaStatus = 3;
        antennaStatus = atoi(antenna.value());
        if (antennaStatus == 3) { // three means antenna detected
          delay(300);
          blinkDelay(10, 10);
        }
#endif

        // no NMEA is a problem. Blink five times in a second.
      } else {
        blinkDelay(5, 100);
      } // end isvalid
    } //end fixage

    // Force LED off. Shouldn't be required, but...
    digitalWrite(statusLED, LOW);
  }

  // reset the watchdog timer
  wdt_reset();
} //end loop


