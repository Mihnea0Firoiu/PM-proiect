#include <Arduino.h>
const int adcPin = A0;
  
// calculate your own m using ph_calibrate.ino
// When using the buffer solution of pH4 for calibration, m can be derived as:
 // m = (pH7 - pH4) / (Vph7 - Vph4)
 // pH7
 // pH4
 // Vph7 2.9
 // Vph4 3.3
const float m = -7.63358; 
const float actualVref = 4.64; // Measure this with your multimeter between 5V and GND pins

void setup()
{
   Serial.begin(115200);
}

void loop() 
{
   float Po = analogRead(adcPin) * actualVref / 1023;
   float phValue = 7 - (2.535 - Po) * m;
   Serial.print("p h value = "); 
   Serial.println(phValue);
   delay(5000);
}
