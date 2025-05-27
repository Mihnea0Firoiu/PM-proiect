#define pH_PIN A0

// Calibration values - ADJUST THESE AFTER CALIBRATION
float calibph7 = 2.9; // Voltage reading when probe is in pH 7.01 buffer
float calibph4 = 3.3; // Voltage reading when probe is in pH 4.01 buffer

// Calculated calibration constants
float m; // slope
float b; // y-intercept

void setup() {
  Serial.begin(115200);
  Serial.println("pH Sensor Calibration Started");
  
  // Calculate slope and intercept for linear equation
  // pH = m * Voltage + b
  m = (4.01 - 7.01) / (calibph4 - calibph7);
  b = 7.01 - m * calibph7;
  
  Serial.println("Calibration Parameters:");
  Serial.print("Slope (m): ");
  Serial.println(m, 4);
  Serial.print("Intercept (b): ");
  Serial.println(b, 4);
  Serial.println("pH readings will start in 3 seconds...");
  delay(3000);
}

void loop() {
  // Take multiple readings for stability
  float adcSum = 0;
  int numReadings = 10;
  
  for(int i = 0; i < numReadings; i++) {
    adcSum += analogRead(pH_PIN);
    delay(50);
  }
  
  float avgADC = adcSum / numReadings;
  
  // Convert ADC to voltage (0-1023 maps to 0-5V)
  float voltage = avgADC * 4.64 / 1024.0;
  
  // Calculate pH using calibration equation
  float phValue = m * voltage + b;
  
  // Display results
  Serial.print("ADC: ");
  Serial.print(avgADC, 0);
  Serial.print("  Voltage: ");
  Serial.print(voltage, 3);
  Serial.print("V  pH: ");
  Serial.println(phValue, 2);
  
  delay(1000);
}

/*
CALIBRATION PROCEDURE:
1. Connect pH sensor to Arduino (VCC->5V, GND->GND, Signal->A0)
2. Make sure BOTH grounds are connected properly!
3. Upload this code with initial calibph7 and calibph4 values
4. Place probe in pH 7.01 buffer solution
5. Adjust potentiometer on pH board until voltage reads ~2.50V
6. Record the actual voltage reading -> update calibph7 value
7. Rinse probe with distilled water
8. Place probe in pH 4.01 buffer solution  
9. Record voltage reading -> update calibph4 value
10. Update code with actual voltage values and re-upload
11. Your pH readings should now be accurate!

TIPS:
- Use fresh calibration buffers (pH 4.01 and 7.01)
- Wait for readings to stabilize before recording values
- Store electrode in 3M KCl solution when not in use
- Recalibrate periodically for best accuracy
- Consider using ADS1115 for 16-bit ADC precision vs 10-bit Arduino ADC
*/