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
  COMMAND_FORWARD = 0,
  COMMAND_REVERSE = 1,
  COMMAND_TURN_LEFT = 2,
  COMMAND_TURN_RIGHT = 3,
  COMMMAND_SETTING_1 = 4,
  COMMMAND_SETTING_2 = 5,
  COMMMAND_SETTING_3 = 6,
  COMMAND_ULTRASONIC = 7,
  COMMAND_COLOUR = 8,
  COMMAND_CAMERA = 9,
  COMMAND_SERVO_OPEN = 10,
  COMMAND_SERVO_CLOSE = 11,
  COMMAND_GET_STATS = 12,
  COMMAND_CLEAR_STATS = 13,
  COMMAND_STOP = 14
} TCommandType;
#endif

