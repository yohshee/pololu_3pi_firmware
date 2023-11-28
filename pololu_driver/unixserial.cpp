/*
 * Serial port class, intended for use with Qt. Since it uses QString.
 *
 * Author: Rick Coogle
 */


#include "unixserial.h"


UnixSerial::UnixSerial()
{
    fdSerial = -1;
    strDevice = "";

    bActive = false;
    bConfig = false;

    iTimeoutVal = SERIAL_TIMEOUT_USECS;

    memset(&curtio, 0, sizeof(struct termios));
    memset(&oldtio, 0, sizeof(struct termios));
}

UnixSerial::~UnixSerial()
{
    CloseDevice();
}

// Opens a device
bool UnixSerial::OpenDevice(QString strName)
{
    QByteArray baDevName;

    // Make sure we don't already have an open device;
    // in this case, just attempt to close.
    CloseDevice();

    baDevName = strName.toLocal8Bit();
    fdSerial = open(baDevName.data(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    iLastErrno = errno;

    if(fdSerial < 0)
      return false;

    strDevice = strName;
    bActive = true;

    return true;
}

// Closes a device, restoring the original configuration,
// if done.
void UnixSerial::CloseDevice()
{
    if(fdSerial < 0)
        return;

    RestoreSettings();
    close(fdSerial);
    iLastErrno = errno;

    fdSerial = -1;
    bActive = false;
}

// Configures the serial port.
void UnixSerial::Configure(speed_t serialSpeed)
{
  if(fdSerial < 0)
      return;

  // Save the original parameters off
  tcgetattr(fdSerial, &oldtio);

  // Clear parameters
  memset(&curtio, 0, sizeof(struct termios));

  // Set to 8N1
  curtio.c_cflag |= CS8 | CLOCAL | CREAD;

  // Use raw input...
  curtio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                      INLCR | PARMRK | INPCK | ISTRIP | IXON);

  // And raw output...
  curtio.c_oflag = 0;

  // With no line processing.
  curtio.c_lflag &=  ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Set control characters
  curtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
  curtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
  curtio.c_cc[VERASE]   = 0;     /* del */
  curtio.c_cc[VKILL]    = 0;     /* @ */
  curtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
  curtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
  curtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
  curtio.c_cc[VSWTC]    = 0;     /* '\0' */
  curtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
  curtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
  curtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
  curtio.c_cc[VEOL]     = 0;     /* '\0' */
  curtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
  curtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
  curtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
  curtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
  curtio.c_cc[VEOL2]    = 0;     /* '\0' */

  // Set communication speeds
  cfsetispeed(&curtio, serialSpeed);
  cfsetospeed(&curtio, serialSpeed);

  // Flush the line and set it up
  tcflush(fdSerial, TCIOFLUSH);
  tcsetattr(fdSerial, TCSANOW, &curtio);
  tcflush(fdSerial, TCIOFLUSH);

  bConfig = true;
}

// Restores an old port configuration. Does nothing
// if the port hasn't been configured from the object.
void UnixSerial::RestoreSettings()
{
    if(fdSerial < 0)
        return;

    if(bConfig) {
        tcsetattr(fdSerial, TCSANOW, &oldtio);
        bConfig = false;
    }
}

// Transmits a buffer over the wire.
bool UnixSerial::SendBuffer(char* pszBuf, int iSize)
{
    int iRet;

    if(!IsActive())
        return false;

    // Write the whole buffer to the wire
    iRet = write(fdSerial, pszBuf, iSize);
    iLastErrno = errno;
    if(iRet < 0 || (iRet == 0 && iSize > 0))
        return false;

    return true;
}

// And this one receives a buffer from the wire.
bool UnixSerial::GetBuffer(char* pszBuf, int iSize, bool bEndOnNewline)
{
    int iLength;
    int iReadBytes;
    int iElapsedTime;
    struct timeval startTime, curTime;

    if(!IsActive())
        return false;

    iReadBytes = 0;
    iLength = 0;
    gettimeofday(&startTime, NULL);

    // Now, keep reading until we have a full buffer
    while(iLength < iSize) {
        iReadBytes = read(fdSerial, &pszBuf[iLength], iSize - iLength);
        iLastErrno = errno;
        if(iReadBytes < 0) {
            errno = 0;
            iReadBytes = 0;
        }
        iLength += iReadBytes;

        if(bEndOnNewline) {
            if(pszBuf[iLength - 1] == '\n')
                break;
        }

        gettimeofday(&curTime, NULL);
        iElapsedTime = (curTime.tv_sec - startTime.tv_sec)*1000000 +
                curTime.tv_usec - startTime.tv_usec;

        if(iElapsedTime >= iTimeoutVal) {
            // Timed out!
            return false;
        }
    }

    return true;
}
