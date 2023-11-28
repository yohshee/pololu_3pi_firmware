/*
 * Simple class that abstracts out control of a Pololu programmed
 * with our firmware that makes it behave like a K-Team Khepera.
 * This is obviously the header file.
 *
 * Author: Rick Coogle
 */


#ifndef POLOLUROBOT_H
#define POLOLUROBOT_H

#include <QString>
#include <unixserial.h>

#define POLOLU_BUFFER_LEN 256
#define POLOLU_WHEELBASE  7.5
#define POLOLU_RES        0.333333
#define POLOLU_MAX_SENSOR 4000

class PololuRobot
{

public:
    PololuRobot();
    ~PololuRobot();

    // robot connection management
    int  OpenRobot(QString strRobotDev);
    void CloseRobot();
    bool PingRobot();

    // robot properties
    int GetLastId()                    { return iCurId; }
    int GetWaitTime()                  { return iWaitTime; }
    void SetWaitTime(int iNewTime)     { iWaitTime = iNewTime; }
    int GetTimeout()                   { return serial.GetTimeout();   }
    void SetTimeout(int iNewTimeout)   { serial.SetTimeout(iNewTimeout); }
    int GetScaleFactor()               { return iScaleFactor; }
    void SetScaleFactor(int iNewScale) { iScaleFactor = iNewScale; }
    bool IsActive()                    { return serial.IsActive(); }

    // high-level robot commands
    int GetLinePos(int *piLine);
    int SetVel(double dVel, double dTurnRate);
    int Stop();

    // low-level robot commands
    int GetSpeed(int* piLeft, int* piRight);
    int GetPosition(int* piLeftCount, int* piRightCount);
    int GetFloorSensors(int* piValues, int iLength);
    int GetCharge(int *piVoltage);

    int SetSpeed(int iLeft, int iRight);
    int SetPosition(int iLeftCount, int iRightCount);

private:
    // helper methods
    int ExecuteCommand(char chCmd, int iParamCount, int* pParams, int iRetValCount,
                       int* pRetVals);

    // member variables
    int iCurId;
    int iWaitTime;
    int iScaleFactor;

    UnixSerial serial;
};

#endif // POLOLUROBOT_H
