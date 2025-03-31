#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X); 
//Gain (1x, 4x, 16x, 60x)
//Integration Time (2.4ms–700ms)
//Detection Distance: 1–3 cm ideal
//Black Paper Enclosure: Fully enclosed (no light leaks)
//Object Surface: Matte (non-shiny) colors work best
void setup() {
  Serial.begin(9600);
  if (!tcs.begin()) {
    Serial.println("Sensor error!");
    while (1);
  }
}

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  
  Serial.print("R:"); Serial.print(r);
  Serial.print(" G:"); Serial.print(g);
  Serial.print(" B:"); Serial.print(b);
  Serial.print(" Clear:"); Serial.println(c); // Clear channel (ambient light check)
  
  delay(500);
}
//Color Thresholds: R/G/B > 40% of total light

/*
Ideal Raw Readings for TCS34725 (Enclosed with Black Paper)
For reliable color detection, aim for these raw RGB values (from tcs.getRawData(&r, &g, &b, &c)):

Condition	Ideal Raw Range (R, G, B)	Notes
Red Object	R > 10,000, G/B < 5,000	Red should dominate
Green Object	G > 10,000, R/B < 5,000	Green should dominate
Blue Object	B > 10,000, R/G < 5,000	Blue should dominate
No Object	All values < 1,000	Low readings = no object detected
Saturation	Any value = 65,535	Sensor is overwhelmed (reduce gain/integration time)
*/
/*
2. Step-by-Step Adjustment Process
Step 1: Start with Default Settings
Use the Adafruit TCS34725 library and initialize with default values:
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_16X);

Step 2: Test Raw Readings
Open Serial Monitor and observe R, G, B values.


Goal: Values should be between 5,000–50,000 for dominant colors.


Step 3: Tune Based on Readings
Too low? → Increase gain (60x) or integration time (700ms).


Saturated? → Reduce gain (4x) or integration time (50ms).


Step 4: Refine Color Detection Logic
Instead of using raw values, normalize readings for better accuracy:
float total = r + g + b;
float redPercent = (r / total) * 100;
if (redPercent > 40) Serial.println("RED");

Step 5: Calibrate for Your Objects
Run a calibration sequence using known red, green, blue objects.


Save reference values for each color and adjust thresholds accordingly.


*/