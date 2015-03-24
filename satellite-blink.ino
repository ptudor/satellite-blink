/*

PA6H Satellite Blinker for Arduino
MIT License

Copyright (c) 2015 Patrick Tudor

This is the software for an ATTiny85 on a PCB I make that is a replacement
for the default GPS puck from Byonics for the MicroTrak RTG and similar.

The code does two things:

1) Connect to the GPS at 9600 default and drop it to 4800 before
   updating the list of NMEA sentences to output, (configurable; non-default)

2) Parse the NMEA stream and twice a minute or so blink an LED with
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


// ATMEL ATTINY45 / ARDUINO
//
//                           +-\/-+
//  Ain0  RST  (D  5)  PB5  1|    |8   VCC
//  Ain3       (D  3)  PB3  2|    |7   PB2  (D  2)  INT0  Ain1   SCK
//  Ain2       (D  4)  PB4  3|    |6   PB1  (D  1)        pwm1   MISO
//                     GND  4|    |5   PB0  (D  0)        pwm0   MOSI
//                           +----+

#define PABY_VERSION "PA6H Monitor v1.3"
#define PABY_COPYRIGHT "(c) Patrick Tudor"

// sleep for this many seconds. less than 255 please.
#define SLEEP_INTERVAL 25
// this is the time after sleep spent reading NMEA in milliseconds.
// unsigned int so please don't exceed around 60,000, or change to long below. 3<N<10 seconds is okay.
#define NMEA_INTERVAL 5000
// if the Oscillator has been calibrated, set this true to pull OSCCAL from EEPROM
// Because of SoftwareSerial, this is pretty much required. But values vary widely.
#define ATTINY_CALIBRATED 1
// if using the MTK3339/PA6H chipset, set to one.
#define GPS_PA6H 1
// this should change 9600 to 4800 but I suggest just using 9600
#define GPS_PA6H_PREFER_4800 0
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
#include <avr/power.h>  // power utilities
#include <avr/wdt.h>    // watchdog timer
#include <avr/sleep.h>  // sleep_mode
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

// enables SBAS search with PMTK_API_SET_SBAS_ENABLED, 0 off 1 on
#define PA6H_SBAS_ENABLE "$PMTK313,1*2E" 
// enables FAA WAAS with PMTK_API_SET_DGPS_MODE, 0 off, 1 RTCM, 2 WAAS
#define PA6H_WAAS_ENABLE "$PMTK301,2*2E" 


// http://www.hhhh.org/wiml/proj/nmeaxor.html
// "The checksum is simple, just an XOR of all the bytes between the $ and the * (not including the delimiters themselves), and written in hexadecimal."
#define PA6H_MESSAGES_DEFAULT "$PMTK314,-1*04"
/* 

GPGLL: only lat and long. No course, no speed, no counters.
"Contains just position and time"

GPRMC: Recommended minimum specific GPS/Transit data: lat/long, speed over ground, course made good, variation
"Contains the minimum data of time, position, speed and course"

GPVTG: course over ground, in degrees, true north, speed in knots and Km
"Contains the course and speed over the ground"

GPGGA: time,latitude,hemisphere,longitude,hemisphere,fix,satellitecount,hdop,alt,msl,dgpstime,dgpsstation
"Contains the essential fix data which provide location and accuracy"

GPGSA: list of active satellites, twelve per line
"Contains data on the Dilution of Precision (DOP) and which satellites are used"

GPGSV: satview, four per line: lines,line,total,id,ele,azi,strength,repeat,repeat,repeat
"Contains the satellite location relative to the receiver and its signal
 to noise ratio. Each message can describe 4 satellites so multiple messages
 may be output depending on the number of satellites being tracked."

GPGRS, GPGST: 
*/

// order: GLL, RMC, VTG, GGA, GSA, GSV, GRS, GST
// byonics default: "NMEA sentences sent each second: $GPGSA, $GPRMC, $GPGGA, $GPGSV."
#define PA6H_MESSAGES "$PMTK314,3,1,3,1,2,6,0,0,0,0,0,0,0,0,0,0,0,0,0*2C"
#define PA6H_BAUD_4800 "$PMTK251,4800*14"
#define PA6H_BAUD_9600 "$PMTK251,9600*17"

static const uint16_t GPS96 = 9600;
static const uint16_t GPS48 = 4800;
//static const uint32_t GPSBaud = 4800;

#if PCB_A
// The serial connection to the GPS device
static const byte RXPin = 3, TXPin = 4;
SoftwareSerial ss(RXPin, TXPin);
// the LED pin
static const byte statusLED = 2;
#endif

// a counter for nmea_read() duration between sleeps.
uint16_t loop_counter;
volatile byte watchdog_counter;

// watchdog interrupt
ISR(WDT_vect) {
  // increment counter once per second
  watchdog_counter++;
}

// take a count, like 4, and a millsecond time, like 1000, and blink the LED.
void blinkDelay(byte count, int timecount) {
  for (int x = 0 ; x < count ; x++)
  {
    // LED on,
    digitalWrite(statusLED, HIGH);
    delay(timecount);
    // LED off,
    digitalWrite(statusLED, LOW);
    delay(timecount);
  }
}

void setup() {

#if ATTINY_CALIBRATED
  // decimal. in range 140 to 190.
  byte calibrationValue = 157; // 157, 149, 166, 155, 185, 183, 182, 157, 156, 170, 165, 150
  // wrapped in if so I can upload code once without doing a write Every Single Time
  if (EEPROM.read(0) != calibrationValue) {
    EEPROM.write(0, calibrationValue);
  }
  // Use TinyTuner "Save_to_EEPROM" first to tune Oscillator
  OSCCAL =  EEPROM.read(0), HEX;
#endif

  // begin serial at 9600 default to set 4800
  ss.begin(GPS96);

  // set pin modes
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, LOW);
  pinMode(RXPin, INPUT);
  pinMode(TXPin, OUTPUT);
  // give a "power on" blink, six times with 50ms delay
  blinkDelay(6, 50);
  
  // just another delay to let the GPS boot up
  delay(400);
  
#if GPS_PA6H_PREFER_4800
  blinkDelay(4, 75);
  ss.println(PA6H_BAUD_4800);
  ss.end();
  // begin serial at 4800
  ss.begin(GPS48);
#endif

  // push custom message interval list
#if GPS_PA6H
  //ss.println(PA6H_MESSAGES_DEFAULT);
  ss.println(PA6H_MESSAGES);
  delay(50);
  ss.println(PA6H_SBAS_ENABLE);
  delay(50);
  ss.println(PA6H_WAAS_ENABLE);
  delay(50);
#endif

  // uses too much RAM. Don't enable.
#if ANTENNA
  ss.println(PA6H_ANTENNA_STATUS);  //softwareserial
#endif

  // print the OSCCAL value. Probably around 0x80.
#if DEBUG
  ss.print(F("OSCCAL:"));
  ss.println(EEPROM.read( 0 ), HEX);
#endif

  // give the first run a headstart
  watchdog_counter = SLEEP_INTERVAL;

  // conserve energy
  power_adc_disable();
  power_usi_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();
  setup_watchdog(6); //Setup watchdog to go off after 1sec

}

void loop() {

  sleep_mode(); // sleep until interrupt that increments counter
  if (watchdog_counter > SLEEP_INTERVAL) {
    nmea_read();
    watchdog_counter = 0;
  }

}

void nmea_read() {
  // quick blink when we wake up
  blinkDelay(1, 100);

  while (loop_counter < NMEA_INTERVAL) {
    // read NMEA into gps object
    gps.encode(ss.read());

    // increment counter toward NMEA_INTERVAL
    loop_counter++;
    // slow it down just a little tiny bit
    delay(1);
  }

  // when we cross the threshold, update LED status
  // let's reset the counter
  loop_counter = 0;
  // notify user an update is arriving with a quick quarter-second blink
  blinkDelay(4, 60);
  // and pause for a second
  delay(1000);

  // let's make sure that we have fresh data
  uint32_t fixage;
  fixage = gps.satellites.age();

  // average is in the range 1000-3000. 10 seconds is too long. "no data" is -1 unsigned or 4B.
  if (fixage > 10000) {
#if DEBUG
    ss.print(F("age:"));
    ss.println(fixage);
#endif
    // no data, solid LED for four seconds.
    // the most likely cause for this is an untuned oscillator.
    blinkDelay(1, 4000);
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
  //  }

} //end loop

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {

  if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings

  byte bb = timerPrescaler & 7;
  if (timerPrescaler > 7) bb |= (1 << 5); //Set the special 5th bit if necessary

  //This order of commands is important and cannot be combined
  MCUSR &= ~(1 << WDRF); //Clear the watch dog reset
  WDTCR |= (1 << WDCE) | (1 << WDE); //Set WD_change enable, set WD enable
  WDTCR = bb; //Set new watchdog timeout value
  WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}
