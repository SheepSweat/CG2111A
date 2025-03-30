#include "AFMotor.h"



// Motor control
#define FRONT_LEFT   4 // M4 on the driver shield
#define FRONT_RIGHT  1 // M1 on the driver shield
#define BACK_LEFT    3 // M3 on the driver shield
#define BACK_RIGHT   2 // M2 on the driver shield


AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

void move(float speed, int direction)
{
  int speed_scaled = (speed/100.0) * 255;
  motorFL.setSpeed(speed_scaled);
  motorFR.setSpeed(speed_scaled);
  motorBL.setSpeed(speed_scaled);
  motorBR.setSpeed(speed_scaled);

  switch(direction)
    {
      case BACK:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case GO:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case CW:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case CCW:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case STOP:
      default:
        motorFL.run(STOP);
        motorFR.run(STOP);
        motorBL.run(STOP);
        motorBR.run(STOP); 
    }
}

void forward(float dist, float speed)
{
  if(dist > 0){
    deltaDist = dist; //
  }
  else{
    stop();
    dbprintf("deltadist is 0_f");
  }

  newDist=forwardDist + deltaDist;
  dir=(TDirection) FORWARD;
  move(speed, FORWARD);
}

void backward(float dist, float speed)
{
  if(dist > 0){
    deltaDist = dist;
  } 
  else{
    stop();
    dbprintf("deltadist is 0_b");
  }

  newDist=reverseDist + deltaDist;
  dir=(TDirection) BACKWARD;
  move(speed, BACKWARD);
}

void ccw(float dist, float speed)
{
  dir=(TDirection) LEFT;
  move(speed, CCW);
}

void cw(float dist, float speed)
{
  dir=(TDirection) RIGHT;
  move(speed, CW);
}
void servo_angle(const int left_angle, const int right_angle) { //20,160
  if(left_angle>180 || left_angle<0 || right_angle>180 ||right_angle<0){
    return; // do it again
  }
  // Convert angle (0-180) to PWM value (1000-2000 for ~1ms-2ms)
   OCR5A = 1000 + (left_angle * 1000 / 180);  
   OCR5B = 1000 + (right_angle * 1000 / 180);
}

void stop()
{
  dir=(TDirection) STOP;
  move(0, STOP);
  newDist=0;
  deltaDist=0;
}

