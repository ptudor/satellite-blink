# satellite-blink
ATTiny85 code to blink the count of GNSS satellites in view from a NMEA stream

Requires TinyGPS++ Library.

### TinyGPS++
[Downloads](https://github.com/mikalhart/TinyGPSPlus/releases)
[Documentation](http://arduiniana.org/libraries/tinygpsplus/)

### Tune Oscillator

```Arduino
// http://forum.arduino.cc/index.php?topic=153034.15

#include <SoftwareSerial.h>

SoftwareSerial mySerial(1,2); // RX, TX

void setup()  
{
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
  //OSCCAL = 0x5A;
}

void loop() // run over and over
{
char dir = 1;
int d = 0;
while (1) {
  OSCCAL += dir;
  d = OSCCAL;
  mySerial.print("\nThis is SoftwareSerial with OSCCAL=");
  mySerial.println(d);
  delay(300);
  if (OSCCAL == 0) dir = 1;
  if (OSCCAL == 255) dir = -1;
}
}
```
