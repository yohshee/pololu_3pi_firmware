/*
 * Serial port class header, intended for use with Qt. Since it uses QString.
 *
 * Author: Rick Coogle
 */


#ifndef UNIXSERIAL_H
#define UNIXSERIAL_H

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#include <unistd.h>

#include <QString>

// Convenience constants
#define DEFAULT_SPEED        B9600
#define SERIAL_TIMEOUT_USECS 1000000


class UnixSerial
{
public:
    UnixSerial();
    ~UnixSerial();

    // Member functions

    // Device setup
    bool OpenDevice(QString strName);
    void CloseDevice();
    void Configure(speed_t serialSpeed);
    void RestoreSettings();

    // Transmission/Reception
    bool SendBuffer(char* pszBuf, int iSize);
    bool GetBuffer(char* pszBuf, int iSize, bool bEndOnNewline);

    // Status
    bool IsActive() { return bActive; }
    int LastError() { return iLastErrno; }

    // Other
    int GetTimeout()                 { return iTimeoutVal;   }
    void SetTimeout(int iNewTimeout) { iTimeoutVal = iNewTimeout; }

private:
    bool bActive;   // is this object holding an active serial port
    bool bConfig;   // has Configure() been called on the object?
    int iLastErrno; // last value of errno after a method that invokes a system call
    int iTimeoutVal; // receive timeout

    QString strDevice; // serial device name

    int fdSerial;           // serial port file descriptor
    struct termios oldtio;  // previous terminal settings
    struct termios curtio;  // current terminal settings
};

#endif // UNIXSERIAL_H
