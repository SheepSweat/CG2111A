
#include <serialize.h>
#include "packeto.h"
#include "constanto.h"
#include <stdarg.h>
#include <math.h>
#include <avr/io.h>

/*
 * Alex's configuration constants
 */
volatile TDirection dir;

#define ALEX_LENGTH 26
#define ALEX_BREADTH 15

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      4//edit pls

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC  20.42 
//#define PI 3.1415926
/*
 *    Alex's State Variables
 */
volatile float alexDiagonal;
volatile float alexCirc;
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


/*
 * 
 * Alex Communication Routines.
 * 
 */
void left(float ang, float speed){
  if(ang == 0)
  deltaTicks=99999999;

  else
  deltaTicks=computeDeltaTicks(ang);

  targetTicks = leftReverseTicksTurns + deltaTicks;

  ccw(ang,speed);
}
void right(float ang, float speed){
  if(ang == 0)
  deltaTicks=99999999;
  else
  deltaTicks = computeDeltaTicks(ang);
  
  targetTicks = rightReverseTicksTurns + deltaTicks;

  cw(ang,speed);
}

// New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang)
{
    // We will assume that angular distance moved = linear distance moved in one wheel
    // revolution. This is probably incorrect but simplifies calculation.
    // # of wheel revs to make one full 360 turn is vincentCirc / WHEEL_CIRC
    // This is for 360 degrees. For ang degrees it will be (ang * vincentCirc) / (360 * WHEEL_CIRC)
    // To get ticks, we multiply by COUNTS_PER_REV.

    unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

    return ticks;
}
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
    TPacket statusPacket; // Create new packet
    statusPacket.packetType = PACKET_TYPE_RESPONSE; // Set packet type
    statusPacket.command= RESP_STATUS; // Set command field
    
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
void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}
void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
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
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD&=~(0b00001100); //input for PD2 & 3
  PORTD|=0b00001100;  //pullup
}

ISR(INT3_vect){
  leftISR();
}

ISR(INT2_vect){
  rightISR();
}
// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if(dir==FORWARD){
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }else if(dir==BACKWARD){
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }else if(dir==LEFT){
    leftReverseTicksTurns++;
    
  }else if(dir==RIGHT){
    leftForwardTicksTurns++;
    
  }

  //Serial.print("LEFT: ");
  //Serial.println(leftTicks);
}

void rightISR()
{
  if(dir==FORWARD){
    rightForwardTicks++;
  }else if(dir==BACKWARD){
    rightReverseTicks++;
  }else if(dir==LEFT){
    rightForwardTicksTurns++;
    
  }else if(dir==RIGHT){
    rightReverseTicksTurns++;
   
  }

  //Serial.print("RIGHT: ");
  //Serial.println(rightTicks);
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EIMSK=0b00001100; //both int2 & int3 enabled
  EICRA=0b10100000; //both falling edge INT2 & INT3
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
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
  targetTicks = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
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

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
      break;

    /*
     * Implement code for other commands here.
     * 
     */
     case COMMAND_REVERSE:
        sendOK();
        backward((double)command->params[0], (float) command->params[1]);
      break;

     case COMMAND_TURN_LEFT:
        sendOK();
        left((double) command->params[0], (float) command->params[1]);
      break;

     case COMMAND_TURN_RIGHT:       
        sendOK();
        right((double) command->params[0], (float) command->params[1]);
      break;

     case COMMAND_SERVO:       
        sendOK();
        servo_angle((int)command->params[0], (int)command->params[1]);
      break;

     case COMMAND_STOP:
        sendOK();
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

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void init_servo(){
  // Set Pin 45 (PL4) and Pin 46 (PL3) as output
  DDRH |= (1 << PH6);  // PH6-PIN9 OC2B
  DDRB |= (1<<PB4); // PB4 - PIN10 OC2A

  // Configure Timer5 for Phase Correct PWM mode (ICR5 as TOP)
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20); // Clear on match, phase correct
 

  // Set initial duty cycle (neutral position for servos)
  OCR2A = 140;  // 1.5ms pulse width
  OCR2B = 140;


  // Start Timer5 with a prescaler of 8
  TCCR2B |= (1 << CS22);
} 


void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
 alexCirc = PI * alexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  init_servo();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
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
// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

// forward(0, 100);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
      
  if(deltaDist > 0)
  {
    if(dir==FORWARD)
    {
      if(forwardDist > newDist)
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
    else
      if(dir == BACKWARD)
      {
        if(reverseDist > newDist)
        {
          deltaDist=0;
          newDist=0;
          stop();
        }
      }
    else
      if((Tdir)dir == STOP)
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }

    if (deltaTicks > 0)
    {
      if (dir == LEFT)
      {

          if (leftReverseTicksTurns >= targetTicks)
          {
              deltaTicks = 0;
              targetTicks = 0;
              stop();
          }
      }
      else if (dir == RIGHT)
      {
          if (rightReverseTicksTurns >= targetTicks)
          {
              deltaTicks = 0;
              targetTicks = 0;
              stop();
          }
      }
      else if ((Tdir)dir == STOP)
      {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
      }
    }


 }
