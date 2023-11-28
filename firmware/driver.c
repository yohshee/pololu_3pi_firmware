/*
 * Driver program for the Pololu 3pi that makes it nominally behave
 * like a Khepera robot in terms of commands. It also broadcasts
 * an identifier.
 *
 * Author: Rick Coogle
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <avr/io.h>
#include <avr/fuse.h>
#include <avr/eeprom.h>
#include <pololu/3pi.h>

#include "xbee_const.h"

// Constants
#define BUILD_VERSION   3

//#define VEL_RES  (2.5)
#define VEL_RES        (3.0)
#define WHEELBASE      (7.5)
#define IR_SENS_COUNT  (5)
#define DEFAULT_RADIUS (10.0)
#define MAX_SPEED      (255)
#define QUEUE_SIZE     (5)

#define RUN_TIME       (5000)
#define BUF_SIZE       (128)
//#define BAUD_RATE      (57600)
#define BAUD_RATE      (9600)

#define SEED_ADDR  ((uint16_t*)(0x1F0))
#define ID_ADDR    ((uint8_t*)(0x0F0))

// Typedefs
typedef struct _MEAS {
  int bValid;
  int iId;
  double pos[2];
} MEAS;

typedef struct _ROBOT_STATE {
  unsigned long uLastTick;  // The last sampled value of the tick counter
  int iBattery;             // Current battery voltage in millivolts 
  int bMoving;              // Moving?
  int iLeftSpeed;           // Left wheel speed
  int iRightSpeed;          // Right wheel speed
  unsigned int uLeftCount;  // Left wheel counts
  unsigned int uRightCount; // Right wheel counts
  double dSpeed;            // Linear velocity
  double dAngVel;           // Turn rate/angular velocity
  double dXPos;             // Current X position
  double dYPos;             // Current Y position
  double dHeading;          // Current heading
} ROBOT_STATE;

static ROBOT_STATE curState;
static const unsigned char uBufSize = 64;
static char uBuffer[BUF_SIZE];
static double dFovRadius = DEFAULT_RADIUS;
static uint8_t uId;

static MEAS measQueue[QUEUE_SIZE];
static int iQueueIdx = 0;

static int iLastTargetId = -1;
static double dLastTargetPos[2];
static int bLastValid = 0;
 
// Prototypes
static void CalcSpeeds(double dSpeed, double dAngVel, 
                       int* pLeftSpeed, int* pRightSpeed);
static void CalcTrueSpeeds(int iLeftSpeed, int iRightSpeed, 
                           double* pdSpeed, double* pdAngVel);
static void LimitSpeeds(ROBOT_STATE* pState);
static void UpdateState(void);
static double GenRand(void);
static double PickRand(double dMin, double dMax);
static void ReseedRNG(void);
static unsigned char ReadId(void);
static double CalcDistance(double* pdPoint);
static double CalcDistancePack(double* pdPoint);
static int DetectTarget(double* pdTarget);

static int ProcessPacket(char* pszPacket);

static void InitializeXbeeSerial(void);
static void SyncOnChar(char chSync);
static void TransmitBuffer(char* pszBuf, unsigned char uSize);
static void ReceiveBuffer(char* pszBuf, unsigned char uSize);


// Main, yay!@
int main(void)
{
  unsigned int iFrame;

  /* Initialization */
  time_reset();

  pololu_3pi_init(2000);

  lcd_init_printf();
  lcd_goto_xy(0,1);
  print("init");

  ReseedRNG();
  uId = ReadId();

  InitializeXbeeSerial();
  delay(2000);

  /* Setup the display */
  clear();
  lcd_goto_xy(0, 0);
  printf("id=%u", uId);
  lcd_goto_xy(6, 0);
  printf("v%d", BUILD_VERSION);
  lcd_goto_xy(0, 1);
  print("ready.");

  /* Initial setup prior to loop */
  lcd_goto_xy(0, 1);
  print("run.  ");
  set_motors(0, 0);

  curState.uLeftCount = 0;
  curState.uRightCount = 0;

  //SyncOnChar('\n');
  iFrame = 0;
  curState.uLastTick = get_ticks();
  /* Main loop */
  while(1) {
    if(iFrame % 100 == 0) {
      UpdateState();
      lcd_goto_xy(0,1);
      printf("%d mV", curState.iBattery);
    }
    ReceiveBuffer(uBuffer, BUF_SIZE);
    ProcessPacket(uBuffer);
    iFrame++;
    green_led(TOGGLE);
  }

  set_motors(0, 0);
  green_led(LOW);

  /* Print stop */
  lcd_goto_xy(0, 1);
  print("stop.");

  return 0;
}

// Updates the current state; i.e., performs dead reckoning updates
void UpdateState(void)
{
  unsigned long uCurTick;
  double dTimeStep, dTemp;
  double dDistDelta, dAngDelta;
  
  // Figure out the time
  uCurTick = get_ticks();
  dTimeStep = (double)ticks_to_microseconds(uCurTick - curState.uLastTick);
  curState.uLastTick = uCurTick;

  // Update the state
  dTimeStep = dTimeStep * 1e-6;
  dDistDelta = dTimeStep * curState.dSpeed;
  dAngDelta = dTimeStep * curState.dAngVel;

  curState.dXPos += dDistDelta * cos(curState.dHeading);
  curState.dYPos += dDistDelta * sin(curState.dHeading);
  curState.dHeading += dAngDelta;

  // Fix up the heading to keep it between the lines
  if(curState.dHeading > 2.0*M_PI)
    curState.dHeading -= 2.0*M_PI;
  else if(curState.dHeading < -2.0*M_PI)
    curState.dHeading += 2.0*M_PI;

  // Update the odometry
  dTemp = (WHEELBASE * dAngDelta) / 2.0;

  curState.uLeftCount += (unsigned int)floor(VEL_RES * (dDistDelta - dTemp));
  curState.uRightCount += (unsigned int)floor(VEL_RES * (dDistDelta + dTemp));

  // Battery voltage
  curState.iBattery = read_battery_millivolts_3pi();
}

// Computes the necessary speeds for driving the wheels
// for the given speed and angular velocity. Speed is in
// cm/s and angular velocity is in rad/sec
static
void CalcSpeeds(double dSpeed, double dAngVel, int* pLeftSpeed, 
                int* pRightSpeed)
{
  double dLeft, dRight;
  double dTemp;

  // Sanity check
  if(!pLeftSpeed || !pRightSpeed)
    return;

  dTemp = (WHEELBASE * dAngVel) / 2.0;

  dLeft = VEL_RES * (dSpeed - dTemp);
  dRight = VEL_RES * (dSpeed + dTemp);

  // Cap the speeds
  if(dLeft > 255.0)
    dLeft = 255.0;
  else if(dLeft < -255.0)
    dLeft = -255.0;

  if(dRight > 255.0)
    dRight = 255.0;
  else if(dRight < -255.0)
    dRight = -255.0;

  *pLeftSpeed = (int)floor(dLeft + 0.5);
  *pRightSpeed = (int)floor(dRight + 0.5);
}

// Backs out the angular and linear velocities from the
// motor speed constants.
static
void CalcTrueSpeeds(int iLeftSpeed, int iRightSpeed, 
                    double* pdSpeed, double* pdAngVel)
{
  double dSpeed, dAngVel;
  double dLeft, dRight;
  double dFactor;

  // Sanity check
  if(!pdSpeed || !pdAngVel)
    return;

  dLeft = (double)iLeftSpeed;
  dRight = (double)iRightSpeed;

  dFactor = 1.0/VEL_RES;

  dSpeed = dFactor * (dRight + dLeft) / 2.0;
  dAngVel = dFactor * (dRight - dLeft) / WHEELBASE;

  *pdSpeed = dSpeed;
  *pdAngVel = dAngVel;
}

// Limits the speeds that have already been set in the current
// robot state.
static
void LimitSpeeds(ROBOT_STATE* pState)
{
  if(pState->iLeftSpeed > MAX_SPEED)
    pState->iLeftSpeed = MAX_SPEED;
  else if(pState->iLeftSpeed < -MAX_SPEED)
    pState->iLeftSpeed = -MAX_SPEED;

  if(pState->iRightSpeed > MAX_SPEED)
    pState->iRightSpeed = MAX_SPEED;
  else if(pState->iRightSpeed < -MAX_SPEED)
    pState->iRightSpeed = -MAX_SPEED;
}

// Reads the robot's unique identifier out of eeprom
static
uint8_t ReadId(void)
{
  uint8_t uRet;

  eeprom_busy_wait();
  uRet = eeprom_read_byte(ID_ADDR);

  return uRet;
}

// Generates a random number between zero and one
static
double GenRand(void)
{
  return (double)(rand())/(double)(RAND_MAX);
}

// Picks a random number in the given range
static
double PickRand(double dMin, double dMax)
{
  double dRet;

  dRet = dMin + (dMax - dMin) * GenRand();

  return dRet;
}

// Reseeds the random number generator
static
void ReseedRNG(void)
{
  uint16_t uSeed;

  // First, we need to get the old seed
  eeprom_busy_wait();
  uSeed = eeprom_read_word(SEED_ADDR);

  srand(uSeed);

  // Eh. Do it, uh, simply.
  uSeed++;
  eeprom_busy_wait();
  eeprom_write_word(SEED_ADDR, uSeed);
}

// Computes the distance from the given point to the current
// robot position.
static
double CalcDistance(double* pdPoint)
{
  double dXDiff, dYDiff;
  double dDist;

  if(!pdPoint)
    return NAN;

  dXDiff = pdPoint[0] - curState.dXPos;
  dYDiff = pdPoint[1] - curState.dYPos;
  dDist = sqrt(dXDiff*dXDiff + dYDiff*dYDiff);

  return dDist;
}

// Computes the distance from the start to the end point; points
// are packed into a single array
static
double CalcDistancePack(double* pdPoint)
{
  double dXDiff, dYDiff;
  double dDist;

  if(!pdPoint)
    return NAN;

  dXDiff = pdPoint[0] - pdPoint[2];
  dYDiff = pdPoint[1] - pdPoint[3];
  dDist = sqrt(dXDiff*dXDiff + dYDiff*dYDiff);

  return dDist;
}

static
int DetectTarget(double* pdTarget)
{
  int bResult = 0;
  double dDist;

  dDist = CalcDistancePack(pdTarget);

  if(!isnan(dDist)) {
    if(dDist <= dFovRadius) {
      bResult = 1;

      // Adjust the coordinates with respect to the 
      //  robot position (as given)
      pdTarget[0] -= pdTarget[2];
      pdTarget[1] -= pdTarget[3];

      // Fuzz up the returned targets
      pdTarget[0] += PickRand(0, 0.05);
      pdTarget[1] += PickRand(0, 0.05);
    }
  }

  return bResult;
}

/////////////////////////////////////////////////////////////////////////
// COMMAND FUNCTIONS

// Processes a packet pulled off the serial port. 
// The command set is based on the khepera command set.
static
int ProcessPacket(char* pszPacket)
{
  char cmd, retCmd;
  uint8_t uSendId;
  char* pszTok;
  int bSend;
  int iTargetId;
  unsigned int irArray[IR_SENS_COUNT];
  double targetPos[4];
  static char szRetBuf[BUF_SIZE];

  // Sanity check
  if(!pszPacket)
    return 1;

  // Strip off any newlines.
  if(pszPacket[strlen(pszPacket) - 1] == '\n')
    pszPacket[strlen(pszPacket) - 1] = '\0';
 
  if(pszPacket[strlen(pszPacket) - 1] == '\r')
    pszPacket[strlen(pszPacket) - 1] = '\0';

  bSend = 0;
  uSendId = 0;

  // The first byte is a character indicating the command issued.
  cmd = pszPacket[0];
  retCmd = cmd + 32;
  pszTok = strtok(pszPacket, ",");

  switch(cmd) {
  case 'D': // set speed
    pszTok = strtok(NULL, ",");
    curState.iLeftSpeed = atoi(pszTok);
    pszTok = strtok(NULL, ",");
    curState.iRightSpeed = atoi(pszTok);
   
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }

    LimitSpeeds(&curState);

    //CalcTrueSpeeds(curState.iLeftSpeed, curState.iRightSpeed,
    //               &curState.dSpeed, &curState.dAngVel);
    set_motors(curState.iLeftSpeed,
               curState.iRightSpeed);

    if(curState.iLeftSpeed == 0 && curState.iRightSpeed == 0)
      curState.bMoving = 0;
    else
      curState.bMoving = 1;
    sprintf(szRetBuf, "%c,%d\r\n", retCmd, uId);
    bSend = 1;
    break;
  case 'E': // read speed
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }
    sprintf(szRetBuf, "%c,%d,%d,%d\r\n", 
            retCmd,
            curState.iLeftSpeed,
            curState.iRightSpeed,
            uId);
    bSend = 1;
    break;
  case 'G': // set odometry
    pszTok = strtok(NULL, ",");
    curState.uLeftCount = (unsigned int)atoi(pszTok);
    pszTok = strtok(NULL, ",");
    curState.uRightCount = (unsigned int)atoi(pszTok);
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }

    sprintf(szRetBuf, "%c,%d\r\n", retCmd, uId);
    bSend = 1;
    break;
  case 'H': // read odometry
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }
    sprintf(szRetBuf, "%c,%d,%d,%d\r\n",
            retCmd,
            curState.uLeftCount,
            curState.uRightCount,
            uId);
    bSend = 1;
    break;
  case 'N': // request IR sensor status
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }
    read_line_sensors(irArray, IR_EMITTERS_ON);
    sprintf(szRetBuf, "%c,%u,%u,%u,%u,%u,%d\r\n",
            retCmd,
            irArray[0],
            irArray[1],
            irArray[2],
            irArray[3],
            irArray[4],
            uId);
    bSend = 1;
    break;
  case 'S': // sensor FOV
    pszTok = strtok(NULL, ",");
    dFovRadius = atof(pszTok);
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }
    sprintf(szRetBuf, "%c,%d\r\n", retCmd, uId);
    bSend = 1;
    // Protect against nonsense.
    if(dFovRadius <= 0.0)
      dFovRadius = DEFAULT_RADIUS;

    break;
  case 'P': // target position
    pszTok = strtok(NULL, ",");
    iTargetId = atoi(pszTok);
    pszTok = strtok(NULL, ",");
    targetPos[0] = atof(pszTok);
    pszTok = strtok(NULL, ",");
    targetPos[1] = atof(pszTok);
    pszTok = strtok(NULL, ",");
    targetPos[2] = atof(pszTok);
    pszTok = strtok(NULL, ",");
    targetPos[3] = atof(pszTok);
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }
    sprintf(szRetBuf, "%c,%d\r\n", retCmd, uId);
    bSend = 1;
    // Determine whether or not we can take a measurement
    if(DetectTarget(targetPos)) {
      bLastValid = 1;
      iLastTargetId = iTargetId;
      dLastTargetPos[0] = targetPos[0];
      dLastTargetPos[1] = targetPos[1];

      measQueue[iQueueIdx].bValid = bLastValid;
      measQueue[iQueueIdx].iId = iLastTargetId;
      measQueue[iQueueIdx].pos[0] = dLastTargetPos[0];
      measQueue[iQueueIdx].pos[1] = dLastTargetPos[1];

      iQueueIdx = (iQueueIdx + 1) % QUEUE_SIZE;
    }
    break;
  case 'Q':  // return last measurement
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }

    sprintf(szRetBuf, "%c,%d,%d,%d,%d,%d\r\n", retCmd, 
            bLastValid, iLastTargetId, 
            (int)(dLastTargetPos[0]*100.0), (int)(dLastTargetPos[1]*100.0),
            uId);
    bLastValid = 0;
    bSend = 1;
    break;
  case 'V': // return battery voltage
    // Check ID
    pszTok = strtok(NULL, ",");
    if(pszTok != NULL) {
      uSendId = (uint8_t)atoi(pszTok);
      if(uSendId != uId)
        return 0;
    }

    sprintf(szRetBuf, "%c,%d,%d\r\n", retCmd, 
            curState.iBattery,
            uId);
    bSend = 1;
    break;
  case '\n':
  case '\r':
    // Any extraneous newline characters we ignore.
    break;
  default:
    lcd_goto_xy(0, 1);
    //printf("ic:%c   ", cmd);
    return 1;
  }

  //lcd_goto_xy(0,1);
  //printf("cmd:%c ", cmd);
  if(bSend)
    TransmitBuffer(szRetBuf, strlen(szRetBuf));

  return 0;
}

/////////////////////////////////////////////////////////////////////////
// XBEE FUNCTIONS
//

// Initializes the Xbee serial interface
static
void InitializeXbeeSerial(void)
{
  serial_set_baud_rate(BAUD_RATE);
}

// Transmits a buffer
static
void TransmitBuffer(char* pszBuf, unsigned char uSize)
{
  // Wait for any bytes to be cleared out first
  while(!serial_send_buffer_empty())
    ;

  serial_send(pszBuf, uSize);
}

static
void SyncOnChar(char chSync)
{
   char chBuf;

   chBuf = '\0';

   while(chBuf != chSync)
     serial_receive_blocking(&chBuf, 1, 50000);
}

static
void ReceiveBuffer(char* pszBuf, unsigned char uSize)
{
  unsigned int iCount;

  // While perhaps not that efficient, we're just going to pull
  // bytes off the wire one byte at a time.
  iCount = 0;
  while(iCount < uSize) {
    if(serial_receive_blocking(&pszBuf[iCount], 1, 50000) != 1) {
      if(pszBuf[iCount] == '\n')
        break;

      iCount++;
    }
  }

  if((iCount+1) < uSize)
    pszBuf[iCount+1] = '\0';
}
