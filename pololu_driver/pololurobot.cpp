/*
 * pololurobot.cpp: Implementation of the Pololu robot class.
 *   Some of the philosophy behind it comes from the implementation
 *   of the Khepera driver in Player.
 *
 * Author: Rick Coogle
 */

#include <QString>
#include <QStringList>

#include "unixserial.h"
#include "pololurobot.h"

PololuRobot::PololuRobot()
{
    iCurId = -1;
    iWaitTime = 500;
    iScaleFactor = 100;
}

PololuRobot::~PololuRobot()
{
    // Clean up any open connections
    serial.CloseDevice();
}

//////////////////////////////////////////////////////
// Connection management
//

// Opens a connection to the robot on the given device.
// Returns nonzero on failure; specifically 1 if the device could
// not be opened, and 2 if the device could not be pinged. If the device
// cannot be pinged, it is closed.
int PololuRobot::OpenRobot(QString strRobotDev)
{
    int iRet = 0;
    QByteArray baRobotDev;

    baRobotDev = strRobotDev.toLocal8Bit();

    if(serial.OpenDevice(baRobotDev.data())) {
        serial.Configure(DEFAULT_SPEED);

        if(!PingRobot()) {
            serial.CloseDevice();
            iRet = 2;
        }
    }
    else {
      iRet = 1;
    }

    return iRet;
}

// Removes currently open connections
void PololuRobot::CloseRobot()
{
    serial.CloseDevice();
}

// Pings the robot
bool PololuRobot::PingRobot()
{
    QString strCommand;
    QByteArray baCmd;
    char buffer[POLOLU_BUFFER_LEN];
    bool bSendResult, bRecvResult;

    strCommand = "E\r\n";
    baCmd = strCommand.toLocal8Bit();

    if(serial.IsActive()) {
        // Send a simple position request.
        bSendResult = serial.SendBuffer(baCmd.data(), baCmd.length());
        usleep(5000);
        bRecvResult = serial.GetBuffer(buffer, POLOLU_BUFFER_LEN, true);

        return (bSendResult && bRecvResult);
    }

    return false;
}

//////////////////////////////////////////////////////
// Utility methods
//

// Executes a command. Note that this will set the current
// robot id to that of the currently open robot. All buffers passed
// into this function must be preallocated. Returns nonzero on a problem.
int PololuRobot::ExecuteCommand(char chCmd, int iParamCount, int* pParams, int iRetValCount,
                                int* pRetVals)
{
    int i;
    QString strCommand, strRet;
    QByteArray baCmdBuf;
    QStringList listStrRet;

    char buffer[POLOLU_BUFFER_LEN];

    if(!serial.IsActive())
        return 1;

    // Sanity checks
    if(iParamCount > 0 && !pParams)
        return 2;

    if(iRetValCount > 0 && !pRetVals)
        return 2;

    // First, build the command string.
    strCommand += chCmd;
    strCommand += ',';

    for(i = 0; i < iParamCount; i++) {
        strCommand += QString("%1").arg(pParams[i]);

        if(i < (iParamCount - 1))
            strCommand += ',';
    }

    strCommand += "\r\n";

    // Next, send it and wait a tick.
    baCmdBuf = strCommand.toLocal8Bit();
    serial.SendBuffer(baCmdBuf.data(), baCmdBuf.size());
    usleep(iWaitTime);

    // Get the returned buffer and pull out any return values.
    serial.GetBuffer(buffer, POLOLU_BUFFER_LEN, true);
    strRet = buffer;
    listStrRet = strRet.split(",");

    // The first returned item is the command.
    if(listStrRet.at(0).toLower() != strCommand.at(0).toLower())
        return 3;

    // Rest - 1 are returned parameters...
    if(iRetValCount > 0) {
        if(iRetValCount == 1) {
            pRetVals[0] = listStrRet.at(1).toInt();
        }
        else {
            for(i = 1; i < (listStrRet.size() - 1); i++) {
                if(i < iRetValCount) {
                    pRetVals[i] = listStrRet.at(i).toInt();
                }
            }
        }
    }

    // The last element of what comes back is the id of the
    // robot
    iCurId = listStrRet.at(listStrRet.size() - 1).toInt();

    return 0;
}

//////////////////////////////////////////////////////
// High-level robot commands
//

// Generates the same weighted average as the read_line()
// function in the pololu library
int PololuRobot::GetLinePos(int* piLine)
{
    int i, iRet;
    int iLine, iLineSum;
    int iSensorVals[5];

    if(!piLine)
        return -1;

    iRet = GetFloorSensors(iSensorVals, 5);

    if(!iRet) {
        // Compute yon average.
        iLineSum = 0;
        for(i = 0; i < 5; i++)
            iLineSum += iSensorVals[i];

        iLine = (0*iSensorVals[0] + 1000*iSensorVals[1] + 2000*iSensorVals[2] +
                    3000*iSensorVals[3] + 4000*iSensorVals[4]) / iLineSum;

        *piLine = iLine;
    }
    return iRet;
}

// Sets velocities in terms of linear and angular velocity
int PololuRobot::SetVel(double dVel, double dTurnRate)
{
    int iTransVel;
    int iRotVel;
    int iLeftSpeed, iRightSpeed;

    iTransVel = static_cast<int>(dVel * (1.0/POLOLU_RES) * iScaleFactor);
    iRotVel = static_cast<int>((dTurnRate * (1.0/POLOLU_RES) *
                               POLOLU_WHEELBASE) / 2.0);

    iLeftSpeed = iTransVel - iRotVel;
    iRightSpeed = iTransVel + iRotVel;

    return SetSpeed(iLeftSpeed, iRightSpeed);
}

// Simply stops the robot from moving.
int PololuRobot::Stop()
{
    return SetSpeed(0, 0);
}


//////////////////////////////////////////////////////
// Low-level robot commands - all command functions return nonzero
//   on error.
//

// Returns the current robot speed
int PololuRobot::GetSpeed(int* piLeft, int* piRight)
{
    int iRet;
    int iRetVals[2];

    iRet = ExecuteCommand('E', 0, NULL, 2, iRetVals);

    if(!iRet) {
        *piLeft = iRetVals[0];
        *piRight = iRetVals[1];
    }

    return iRet;
}

// Gets the current odometric counts
int PololuRobot::GetPosition(int* piLeftCount, int* piRightCount)
{
    int iRet;
    int iRetVals[2];

    iRet = ExecuteCommand('H', 0, NULL, 2, iRetVals);

    if(!iRet) {
        *piLeftCount = iRetVals[0];
        *piRightCount = iRetVals[1];
    }

    return iRet;
}

// Gets the value of the sensors pointing at the floor
int PololuRobot::GetFloorSensors(int* piValues, int iLength)
{
    int iRet;

    iRet = ExecuteCommand('N', 0, NULL, iLength, piValues);

    return iRet;
}

// Gets the current battery charge, in mV.
int PololuRobot::GetCharge(int* piVoltage)
{
    int iRet;

    iRet = ExecuteCommand('V', 0, NULL, 1, piVoltage);

    return iRet;
}

// Sets the wheel counts
int PololuRobot::SetPosition(int iLeftCount, int iRightCount)
{
    int iRet;
    int iParamVals[2];

    iParamVals[0] = iLeftCount;
    iParamVals[1] = iRightCount;

    iRet = ExecuteCommand('G', 2, iParamVals, 0, NULL);

    return iRet;
}

// Sets the wheel speeds
int PololuRobot::SetSpeed(int iLeft, int iRight)
{
    int iRet = 0;
    int iParamVals[2];

    iParamVals[0] = iLeft;
    iParamVals[1] = iRight;

    iRet = ExecuteCommand('D', 2, iParamVals, 0, NULL);

    return iRet;
}
