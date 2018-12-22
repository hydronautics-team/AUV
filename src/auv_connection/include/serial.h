#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>     /* UNIX standard function definitions */
#include <fcntl.h>      /* File control definitions */
#include <errno.h>      /* Error number definitions */
#include <termios.h>    /* POSIX terminal control definitions */
#include <sys/ioctl.h>  /* ioctl flags */

enum _serial_parity {
    PARITY_NONE = 0,
    PARITY_EVEN,
    PARITY_ODD
};

unsigned int copyVar(int var);

/** @brief Serial device class.
 *
 * This class providing basic functions for serial port,
 * such as configurating and communicating.
 * This class controls serial devices in non-blocking mode only.
 */
class Serial
{
public:
    Serial();
    Serial(std::string filename, int new_baudrate = 115200, int new_databits = 8, int new_parity = PARITY_NONE, int new_stopbits = 1);
    ~Serial();

    Serial& operator<<(const std::string data);
    Serial& operator<<(const std::vector<uint8_t> data);
    Serial& operator>>(std::string &data);
    Serial& operator>>(std::vector<uint8_t> &data);

    /// Maintaining port state
    bool openPort(std::string filename);
    bool closePort();
    bool configurePort(int new_baudrate = 115200, int new_databits = 8, int new_parity = PARITY_NONE, int new_stopbits = 1);
    bool isOpened();

    /// Communication
    bool writePort(std::string data);
    bool writePort(std::vector<uint8_t> data);
    bool readPort(std::string &data, size_t bytes);
    bool readPort(std::vector<uint8_t> &data, size_t bytes);
    bool flush();
    size_t bytesAvailable();

private:
    /// File descriptor for opened file
    int file;

    /// Hardware parameters of the port.
    int baudrate;
    int databits;
    int parity;
    int stopbits;

    /// Structure for storing baud rates
    struct _bauds {
        int rate;
        speed_t constant;
    };
    static const int baudRates_num = 18;
    static const _bauds baudRates[baudRates_num];
};

#endif // SERIAL_H
