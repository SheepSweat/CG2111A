#ifndef __CONSTANTS_INC__
#define __CONSTANTS_INC__

/* 
 *  This file containts all the packet types, commands
 *  and status constants
 *  
 */

// Packet types
typedef enum
{
  PACKET_TYPE_COMMAND = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_ERROR = 2,
  PACKET_TYPE_MESSAGE = 3,
  PACKET_TYPE_HELLO = 4
} TPacketType;

// Response types. This goes into the command field
typedef enum
{
  RESP_OK = 0,
  RESP_STATUS=1,
  RESP_BAD_PACKET = 2,
  RESP_BAD_CHECKSUM = 3,
  RESP_BAD_COMMAND = 4,
  RESP_BAD_RESPONSE = 5 
} TResponseType;


// Commands
// For direction commands, param[0] = distance in cm to move
// param[1] = speed
typedef enum
{
        COMMAND_FORWARD = 0,    //w
        COMMAND_REVERSE = 1,    //s
        COMMAND_TURN_LEFT = 2,  //a
        COMMAND_TURN_RIGHT = 3, //d
        COMMAND_GEAR_1 = 4,     //f
        COMMAND_GEAR_2 = 5,     //g
        COMMAND_GEAR_3 = 6,     //h
        COMMAND_ULTRA = 7,      //z
        COMMAND_COLOUR = 8,     //c
        COMMAND_CAM = 9,        //x     
        COMMAND_STOP = 10,      //y
        COMMAND_SERVO_OPEN = 11,        //q
        COMMAND_SERVO_CLOSE = 12,       //e
        COMMAND_CLEAR_STATS = 13,       //t
        COMMAND_GET_STATS = 14,  //r
        COMMAND_TURN_AND_OPEN_TRAP = 15, //o
        COMMAND_SHAKE = 16 //p

} TCommandType;
#endif

