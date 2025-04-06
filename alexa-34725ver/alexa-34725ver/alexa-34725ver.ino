#include <Adafruit_TCS34725.h>
#include <serialize.h>
#include "packeto.h"
#include "constanto.h"
#include "stdarg.h"
#include <math.h>
#include <avr/io.h>
#include <Servo.h>


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_4X);
/*
 * Alex's configuration constants
 */
volatile TDirection dir;

#define ALEX_LENGTH 12
#define ALEX_BREADTH 14

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 5  //edit pls

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 20
//#define PI 3.1415926
#define TRIG_PIN 25
#define ECHO_PIN 26
#define LED_PIN 52

#define TCS3200_S0 30   // Choose your pin numbers
#define TCS3200_S1 31   // Choose your pin numbers
#define TCS3200_S2 32   // Choose your pin numbers
#define TCS3200_S3 33   // Choose your pin numbers
#define TCS3200_OUT 34  // Choose your pin numbers

int offset = 0;
/*
 *    Alex's State Variables
 */
volatile float alexDiagonal = 18.44;
volatile float alexCirc = 57.93;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

int angSpeed = 70;       // 0-100% speed (uint8_t if you want memory efficiency)
int distSpeed = 100;     // 0-100% speed (uint8_t if you want memory efficiency)
float targetAngle = 40;  // Target angle in degrees (float for precision)
float targetDist = 5;    // Target distance in cm (volatile if you want ISR safety)

unsigned long lastUltrasonicReport = 0;
unsigned long currentMillis;
unsigned int ultramode = 0;

Servo servo1;  //  pin 9
Servo servo2;  // pin 10
Servo servo3;  // pin 46
/*
 * 
 * Alex Communication Routines.
 * 
 */


//prototype functions
void left(float ang, float speed) {
  if (ang == 0) {
    stop();
    dbprintf("ang 0 detected");
  } else {
    deltaTicks = computeDeltaTicks(ang);

    targetTicks = leftReverseTicksTurns + deltaTicks;
    /*
    Serial.println("THis is the delta ticks:");
    Serial.println(deltaTicks);
    Serial.println("THis is the target ticks:");
    Serial.println(targetTicks);
    */
    //dir=(TDirection) LEFT;
    ccw(ang, speed);
  }
}
void right(float ang, float speed) {
  if (ang == 0) {
    stop();
    dbprintf("ang 0 detected");
  } else {
    deltaTicks = computeDeltaTicks(ang);

    targetTicks = rightReverseTicksTurns + deltaTicks;
    /*
    Serial.println("THis is the delta ticks:");
    Serial.println(deltaTicks);
    Serial.println("THis is the target ticks:");
    Serial.println(targetTicks);*/
    //dir=(TDirection) RIGHT;
    cw(ang, speed);
  }
}

// New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
  // We will assume that angular distance moved = linear distance moved in one wheel
  // revolution. This is probably incorrect but simplifies calculation.
  // # of wheel revs to make one full 360 turn is vincentCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * vincentCirc) / (360 * WHEEL_CIRC)
  // To get ticks, we multiply by COUNTS_PER_REV.

  unsigned long ticks = (unsigned long)(round((ang * alexCirc * 5) / (360.0 * 19.8)));  //(10*94.3*4)/(360*20.42)=0.513
                                                                                        /* Serial.println("THis is the after computing:");
    Serial.println(targetTicks);*/
  return ticks;
}

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;                            // Create new packet
  statusPacket.packetType = PACKET_TYPE_RESPONSE;  // Set packet type
  statusPacket.command = RESP_STATUS;              // Set command field

  // Populate params array
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}
void dbprintf(const char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}
void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 19
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 4 and 3 in DDRD to 0 to make them inputs.
  DDRD &= ~(0b00001100);  //input for PD2 & 3
  PORTD |= 0b00001100;    //pullup
}

ISR(INT3_vect) {
  leftISR();
}

ISR(INT2_vect) {
  rightISR();
}
// Functions to be called by INT2 and INT3 ISRs.
void leftISR() {
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;

  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
  /*
  Serial.print("LEFTturnsreverse: ");
  Serial.println(leftReverseTicksTurns);
  Serial.print("LEFTreverse: ");
  Serial.println(leftReverseTicks);
  */
}

void rightISR() {
  if (dir == FORWARD) {
    rightForwardTicks++;
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
  /*
  Serial.print("RIGHTreverseTurns: ");
  Serial.println(rightReverseTicksTurns);
    Serial.print("RIghtreverse: ");
  Serial.println(leftReverseTicks);
  */
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EIMSK = 0b00001100;  //both int2 & int3 enabled
  EICRA = 0b10100000;  //both falling edge INT2 & INT3
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.


// Implement INT2 and INT3 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

  int count = 0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters() {
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0;
  targetTicks = 0;
}

// Clears one particular counter
void clearOneCounter(int which) {
  clearCounters();
  /*
  switch(which)
  {
    case 0:
      clearCounters();
      break;

    case 1:
      leftTicks=0;
      break;

    case 2:
      rightTicks=0;
      break;

    case 3:
      leftRevs=0;
      break;

    case 4:
      rightRevs=0;
      break;

    case 5:
      forwardDist=0;
      break;

    case 6:
      reverseDist=0;
      break;
  }*/
}
// Intialize Alex's internal states

void initializeState() {
  clearCounters();
}

int getDistance() {  // Initialize sensor
  digitalWrite(TRIG_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Get distance
  int distance_cm = pulseIn(ECHO_PIN, HIGH) * 0.034 / 2 - offset;  //currently 0
  return distance_cm;
}


void colourmode() {
  if (!tcs.begin()) {
    dbprintf("COLOR:INIT_FAIL");
    return;
  }
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Normalize colors to avoid brightness variations
  float rn = (float)r / c;
  float gn = (float)g / c;
  float bn = (float)b / c;

  // Determine dominant color safely
  if (rn > gn && rn > bn && rn > 0.4) {  // 40% or more of total color
    dbprintf("RED");
  } else if (gn > rn && gn > bn && gn > 0.4) {
    dbprintf("GREEN");
  } else {
    dbprintf("NONE");
  }
  digitalWrite(LED_PIN, LOW);
}  //this is for tcs34725



void setupTCS3200() {
  pinMode(TCS3200_S0, OUTPUT);
  pinMode(TCS3200_S1, OUTPUT);
  pinMode(TCS3200_S2, OUTPUT);
  pinMode(TCS3200_S3, OUTPUT);
  pinMode(TCS3200_OUT, INPUT);

  // Set frequency scaling to 20% (you can adjust this)
  digitalWrite(TCS3200_S0, HIGH);
  digitalWrite(TCS3200_S1, LOW);  //HIGH HIGH 100, LOW HIGH 2, HIGH LOW 20

  dbprintf("TCS3200 ready!");
}
/*
void colourmode() {
  // Read red component
  digitalWrite(TCS3200_S2, LOW);
  digitalWrite(TCS3200_S3, LOW);
  delay(100);
  int red = pulseIn(TCS3200_OUT, LOW);
  
  // Read green component
  digitalWrite(TCS3200_S2, HIGH);
  digitalWrite(TCS3200_S3, HIGH);
  delay(100);
  int green = pulseIn(TCS3200_OUT, LOW);
  
  // Read blue component
  digitalWrite(TCS3200_S2, LOW);
  digitalWrite(TCS3200_S3, HIGH);
  delay(100);
  int blue = pulseIn(TCS3200_OUT, LOW);
  
  // Determine dominant color
  if (red < green && red < blue) {  // Lower value means stronger color
    dbprintf("RED");
 //Serial.print("RED");
  } 
  else if (green < red && green < blue) {
    dbprintf("GREEN");
  //   Serial.print("GREEN");
  } 
  else {
    dbprintf("NONE");
   //  Serial.print("NONE");
  }*/
/*
  Serial.print("red ");
 Serial.print(red);

  Serial.print("green ");
   Serial.print(green);

  Serial.print("blue ");
  Serial.print(blue);
  Serial.println(" ");
}*/
// this is for tcs3200

void cammode() {
  ;
}

void handleCommand(TPacket *command) {
  switch (command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward(targetDist, distSpeed);
      break;

    case COMMAND_REVERSE:
      sendOK();
      backward(targetDist, distSpeed);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left(targetAngle, angSpeed);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right(targetAngle, angSpeed);
      break;

    case COMMAND_GEAR_1:
      sendOK();
      distSpeed = 50;
      angSpeed = 70;
      targetAngle = 20;
      targetDist = 2;
      break;

    case COMMAND_GEAR_2:
      sendOK();
      distSpeed = 70;
      angSpeed = 70;
      targetAngle = 40;
      targetDist = 5;
      break;

    case COMMAND_GEAR_3:
      sendOK();
      distSpeed = 100;
      angSpeed = 70;
      targetAngle = 90;
      targetDist = 10;
      break;

    case COMMAND_ULTRA:
      sendOK();
      ultramode ^= 1;
      break;

    case COMMAND_COLOUR:
      sendOK();
      //  colourmode();
      break;

    case COMMAND_CAM:
      sendOK();
      cammode();
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_SERVO_OPEN:
      sendOK();
      //servo_angle(180,0);
      servo1.write(130);
      servo2.write(20);
      dbprintf("servo opened");
      break;

    case COMMAND_SERVO_CLOSE:
      sendOK();
      //        servo_angle(0, 180); //do adjust
      servo1.write(30);
      servo2.write(90);
      dbprintf("servo closed");
      break;

    case COMMAND_TURN_AND_OPEN_TRAP:
      sendOK();
      backward(2, 100);
      left(90, 100);
      backward(2, 100);
      stop();
      servo3.write(0);
      dbprintf("trap opened, start shaking");
      break;

    case COMMAND_SHAKE:
      dbprintf("SHAKING");
      sendOK();
      forward(2, 100);
      backward(2, 100);
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      sendOK();
      break;

    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
      sendOK();
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {


        sendOK();
        exit = 1;
      } else
        sendBadResponse();
    } else if (result == PACKET_BAD) {
      sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  }  // !exit
}
/*
void init_servo(){
  // Set Pin 45 (PL4) and Pin 46 (PL3) as output
  DDRL |= (1 << PL3);  // PH6-PIN9 OC2B//PL3-PIN46-OCR5A
  DDRL |= (1 << PL4); // PB4 - PIN10 OC2A//PL4-PIN45-OCR5B

  // Configure Timer5 for Phase Correct PWM mode (ICR5 as TOP)
  TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << WGM51); // Clear on match, phase correct, mode 10, wgm53&51
  TCCR5B = (1 << WGM53);

  // Set initial duty cycle (neutral position for servos)
  ICR5 = 20000;
  OCR5A = 2000;  // 1.0ms/1.5ms/2.0ms 1000/1500/2000 pulse width // 180deg //left-->right
  OCR5B = 1000; // 1.0ms/1.5ms/2.0ms 1000/1500/2000 pulse width // 0deg  //right-->left

  // Start Timer5 with a prescaler of 8
  TCCR5B |= (1 << CS51);
} 
*/

void setup() {
  // put your setup code here, to run once:
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  //init_servo();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  servo1.attach(9);
  servo2.attach(10);
  servo1.write(130);
  servo2.write(20);
  servo3.write(75);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start with LED off
                               /*
  for(int i=0; i<50 ;i++){
  servo1.write(130);
  servo2.write(20);
  delay(500);
  servo1.write(30);
  servo2.write(90);*/
  /* this is for colour testing
  for(int i=0; i<100; i++){
  ultramode();
  delay(500);
  }
  */
  //this is for ultrasonic testing
  initializeState();
  sei();

  const unsigned long timeout = 20000;  // 20 second timeout
  unsigned long startTime = millis();

  while (!tcs.begin()) {
    dbprintf("Retrying color sensor...\n");
    delay(500);  // More reasonable delay between attempts
    dbprintf("Finding colour sensor");
    if (millis() - startTime >= timeout) {
      dbprintf("ERROR: Color sensor initialization failed after %d seconds\n", timeout / 1000);
      break;
    }
  }
  if (tcs.begin()) {
    dbprintf("TCS34725 initialized successfully!\n");
  } else {
    dbprintf("WARNING: Continuing without color sensor\n");
  }
}

void handlePacket(TPacket *packet) {
  switch (packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  // Uncomment for Step 2 of Activity 3 in Week 8 Studio 2
  // forward(0, 100);

  // Uncomment for Week 9 Studio 2
  // Process incoming packets from the Pi
  TPacket recvPacket;  // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    handlePacket(&recvPacket);  // Triggers handleCommand -> which triggers all action functions
  } else if (result == PACKET_BAD) {
    sendBadPacket();
  } else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }
  //test movements

  //forward(100, 40);
  //backward(20, 40);
  //stop();
  //left(100, 40);

  //right(10, 40);



  //
  // Check if forward/backward movement is complete
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
        dir = (TDirection)STOP;
      }
    } else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
        dir = (TDirection)STOP;
      }
    } else if ((Tdir)dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
      dir = (TDirection)STOP;
    }
  }

  // Check if rotation (left/right) is complete
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        dir = (TDirection)STOP;
        //Serial.println("target ticks reached");
      }
    } else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        dir = (TDirection)STOP;
        //Serial.println("target ticks reached");
      }
    } else if ((Tdir)dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
      dir = (TDirection)STOP;
    }
  }
  // Periodically report ultrasonic distance
  if (ultramode == 1) {
    currentMillis = millis();
    if (currentMillis - lastUltrasonicReport >= 200) {
      int distance = getDistance();
      dbprintf("DIST:%d", distance);
      lastUltrasonicReport = currentMillis;
    }
  }
}