/*
 * TCS3200 Full Calibration Test
 * Determines best gain and integration time settings
 * Pin Configuration:
 * S0=30, S1=31, S2=32, S3=33, OUT=34
 */

#define TCS3200_S0  30
#define TCS3200_S1  31
#define TCS3200_S2  32
#define TCS3200_S3  33
#define TCS3200_OUT 34

// Test parameters
const int testGains[] = {1, 2, 3}; // 2%, 20%, 100%
const int testTimes[] = {50, 100, 200}; // ms

void setup() {
  pinMode(TCS3200_S0, OUTPUT);//freq
  pinMode(TCS3200_S1, OUTPUT);//freq
  pinMode(TCS3200_S2, OUTPUT);
  pinMode(TCS3200_S3, OUTPUT);
  pinMode(TCS3200_OUT, INPUT);

  Serial.begin(9600);
  Serial.println("TCS3200 Calibration Test");
  Serial.println("========================");
}

void loop() {
  Serial.println("\nPlace WHITE reference object and press any key...");
  while(!Serial.available());
  while(Serial.available()) Serial.read();
  
  testAllSettings();

  Serial.println("\nPlace RED test object and press any key...");
  while(!Serial.available());
  while(Serial.available()) Serial.read();
  
  testAllSettings();

  Serial.println("\nTest complete. Review results to choose best settings.");
  while(1); // Stop after full test
}

void testAllSettings() {
  Serial.println("\nGAIN\tTIME(ms)\tRED\tGREEN\tBLUE");
  Serial.println("----------------------------------------");
  
  for(int gain : testGains){
    for(int time : testTimes){
      setGain(gain);
      
      int r = readColor(0, time); // Red
      int g = readColor(1, time); // Green
      int b = readColor(2, time); // Blue
      
      Serial.print(gain==1?"2%":gain==2?"20%":"100%");
      Serial.print("\t");
      Serial.print(time);
      Serial.print("\t\t");
      Serial.print(r);
      Serial.print("\t");
      Serial.print(g);
      Serial.print("\t");
      Serial.println(b);
    }
  }
}

// Core functions --------------------------------------------
int readColor(int filter, int duration) {
  setFilter(filter);
  return getFrequency(duration);
}

void setFilter(int color) {
  switch(color){
    case 0: // Red
      digitalWrite(TCS3200_S2, LOW);
      digitalWrite(TCS3200_S3, LOW);
      break;
    case 1: // Green
      digitalWrite(TCS3200_S2, HIGH);
      digitalWrite(TCS3200_S3, HIGH);
      break;
    case 2: // Blue
      digitalWrite(TCS3200_S2, LOW);
      digitalWrite(TCS3200_S3, HIGH);
      break;
  }
}

void setGain(int level) {
  switch(level){
    case 1: // 2%
      digitalWrite(TCS3200_S0, LOW);
      digitalWrite(TCS3200_S1, HIGH);
      break;
    case 2: // 20%
      digitalWrite(TCS3200_S0, HIGH);
      digitalWrite(TCS3200_S1, LOW);
      break;
    case 3: // 100%
      digitalWrite(TCS3200_S0, HIGH);
      digitalWrite(TCS3200_S1, HIGH);
      break;
  }
}

int getFrequency(int duration) {
  unsigned long pulseCount = 0;
  unsigned long startTime = millis();
  
  while(millis() - startTime < duration) {
    pulseCount += pulseIn(TCS3200_OUT, LOW);
  }
  return pulseCount;
}

