#include "serial.h"

const Serial::_bauds Serial::baudRates[] = {
    { 0, B0 },
    { 50, B50 },
    { 75, B75 },
    { 110, B110 },
    { 134, B134 },
    { 150, B150 },
    { 200, B200 },
    { 300, B300 },
    { 600, B600 },
    { 1200, B1200 },
    { 1800, B1800 },
    { 2400, B2400 },
    { 4800, B4800 },
    { 9600, B9600 },
    { 19200, B19200 },
    { 38400, B38400 },
    { 57600, B57600 },
    // { 76800, B76800 }, // Undeclared identifier?
    { 115200, B115200 }
};

/** @brief Default constructor of the class, does nothing.
  *
  * This is the default constructor of the class, does nothing.
  */
Serial::Serial()
{
    baudrate = 0;
    databits = 0;
    parity = 0;
    stopbits = 0;
    file = 0;
}

/** @brief Overloaded constructor of the class, opens and configures port.
  *
  * This is the overloaded constructor of the class, opens and configures port.
  * @param[in]  filename     Name of the device file.
  * @param[in]  new_baudrate Baudrate of the data stream (use only standart values).
  * @param[in]  new_databits Number of the data bits (5-8).
  * @param[in]  new_parity   No, odd or even parity (use PARITY_NONE, PARITY_EVEN or PARITY_ODD).
  * @param[in]  new_stopbits Number of stop bits (1 or 2).
  */
Serial::Serial(std::string filename, int new_baudrate, int new_databits, int new_parity, int new_stopbits)
{
    openPort(filename);
    configurePort(new_baudrate, new_databits, new_parity, new_stopbits);
}

/** @brief Class destructor.
  *
  * This is the class destructor, it closes the port.
  */
Serial::~Serial()
{
    closePort();
}


/** @brief Overloaded << operator, used to write strings to ports.
  *
  * This is the overloaded << operator, used to write strings to ports.
  * @param[in]  data Data string which will be written to port.
  */
Serial& Serial::operator<<(const std::string data)
{
    writePort(data);
    return *this;
}

/** @brief Overloaded >> operator, used to read all available bytes to strings from ports.
  *
  * This is the overloaded >> operator, used to read all available bytes to strings from ports.
  * @param[in]  data Data string which will be written to port.
  */
Serial& Serial::operator>>(std::string &data)
{
    readPort(data, bytesAvailable());
    return *this;
}

/** @brief Overloaded << operator, used to write strings to ports.
  *
  * This is the overloaded << operator, used to write strings to ports.
  * @param[in]  data Data string which will be written to port.
  */
Serial& Serial::operator<<(const std::vector<uint8_t> data)
{
    writePort(data);
    return *this;
}

/** @brief Overloaded >> operator, used to read all available bytes to strings from ports.
  *
  * This is the overloaded >> operator, used to read all available bytes to strings from ports.
  * @param[in]  data Data string which will be written to port.
  */
Serial& Serial::operator>>(std::vector<uint8_t> &data)
{
    readPort(data, bytesAvailable());
    return *this;
}

/** @brief Try to open device file.
  *
  * This function trying to open device file.
  * @param[in]  filename Name of the device file.
  */
bool Serial::openPort(std::string filename)
{
    file = open(filename.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (file < 0) {
        std::cerr << "Could not open port: " << filename << " // Error: " << file << std::endl;
        file = 0;
        return false;
    }

    return true;
}

/** @brief Check is file opened.
  *
  * This function checks is any device opened.
  */
bool Serial::isOpened()
{
    if(file == 0) {
        return false;
    }
    return true;
}

/** @brief Try to close device file.
  *
  * This function trying to close device file.
  * @param[in]  fd File descriptor of opened device file.
  */
bool Serial::closePort()
{
    if(!isOpened()) {
        std::cerr << "File not opened yet!" << std::endl;
        return false;
    }

    int out = close(file);
    if(out == 0) {
        return true;
    }

    std::cerr << "Could not close port: " << file << " // Error: " << out << std::endl;
    return false;
}

/** @brief Try to close device file.
  *
  * This function trying to close device file.
  * @param[in]  new_baudrate Baudrate of the data stream (use only standart values).
  * @param[in]  new_databits Number of the data bits (5-8).
  * @param[in]  new_parity   No, odd or even parity (use PARITY_NONE, PARITY_EVEN or PARITY_ODD).
  * @param[in]  new_stopbits Number of stop bits (1 or 2).
  */
bool Serial::configurePort(int new_baudrate, int new_databits, int new_parity, int new_stopbits)
{
    /* Is file opened */
    if(!isOpened()) {
        std::cerr << "File not opened yet!" << std::endl;
        return false;
    }
    /* Structure for port options */
    struct termios options;
    /* Getting current options from port */
    tcgetattr(file, &options);
    /* Setting baud rates */
    for(int i=0; i<baudRates_num; i++) {
        if(new_baudrate == baudRates[i].rate) {
            cfsetispeed(&options, baudRates[i].constant);
            cfsetospeed(&options, baudRates[i].constant);
            baudrate = new_baudrate;
            break;
        }
        if(i == baudRates_num-1) {
            std::cerr << "Invalid baud rate!" << std::endl;
            return false;
        }
    }
    /* Bit mask for the databits */
    options.c_cflag &= ~copyVar(CSIZE);
    /* Setting data bits*/
    switch(new_databits) {
    case 5: options.c_cflag |= CS5;
        break;
    case 6: options.c_cflag |= CS6;
        break;
    case 7: options.c_cflag |= CS7;
        break;
    case 8: options.c_cflag |= CS8;
        break;
    default:
        std::cerr << "Invalid data bits number!" << std::endl;
        return false;
    }
    databits = new_databits;
    /* Setting parity */
    switch(new_parity) {
    case PARITY_NONE:
        options.c_cflag &= ~copyVar(PARENB);
        options.c_iflag |= (IGNPAR | ISTRIP);
        break;
    case PARITY_EVEN:
        options.c_cflag |= copyVar(PARENB);
        options.c_cflag &= ~copyVar(PARODD);
        options.c_iflag |= (INPCK | ISTRIP);
        break;
    case PARITY_ODD:
        options.c_cflag |= copyVar(PARENB);
        options.c_cflag |= copyVar(PARODD);
        options.c_iflag |= (INPCK | ISTRIP);
        break;
    default:
        std::cerr << "Invalid parity!" << std::endl;
        return false;
    }
    parity = new_parity;
    /* Setting stop bits*/
    switch(new_stopbits) {
    case 1:  options.c_cflag |= copyVar(CSTOPB);
        break;
    case 2:  options.c_cflag &= ~copyVar(CSTOPB);
        break;
    default:
        std::cerr << "Invalid stop bits number!" << std::endl;
        return false;
    }
    stopbits = new_stopbits;
    /* Choosing raw output */
    options.c_lflag &= ~copyVar(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~copyVar(OPOST);
    /* Write setted options */
    tcsetattr(file, TCSANOW, &options);
    return true;
}

/** @brief Write string to port.
  *
  * This function writes string to port.
  * @param[in]  fd File descriptor of opened device file.
  */
bool Serial::writePort(std::string data)
{
    if(!isOpened()) {
        std::cerr << "File not opened yet!" << std::endl;
        return false;
    }

    unsigned long bytes = data.size();
    char *buf = new char[bytes];
    for(size_t i=0; i<bytes; i++) {
        buf[i] = data[i];
    }
    write(file, reinterpret_cast<void*>(buf), bytes);
    delete[] buf;

    return true;
}

/** @brief Write string to port.
  *
  * This function writes string to port.
  * @param[in]  fd File descriptor of opened device file.
  */
bool Serial::writePort(std::vector<uint8_t> data)
{
    if(!isOpened()) {
        std::cerr << "File not opened yet!" << std::endl;
        return false;
    }

    char *ptr = reinterpret_cast<char*>(data.data());
    unsigned long bytes = data.size();
    char *buf = new char[bytes];
    for(size_t i=0; i<bytes; i++) {
        buf[i] = ptr[i];
    }
    write(file, reinterpret_cast<void*>(ptr), data.size());
    delete[] buf;

    return true;
}

/** @brief Read bytes from port.
  *
  * This function checks is any device opened.
  * @param[out]  &data Link to string in which will be placed data from port.
  * @param[in]   bytes Number of bytes to read.
  */
bool Serial::readPort(std::string &data, size_t bytes)
{
    if(!isOpened()) {
        std::cerr << "File not opened yet!" << std::endl;
        return false;
    }

    if(bytesAvailable() < bytes-1) {
        std::cerr << "There are no that many bytes in file!" << std::endl;
        return false;
    }

    char *buf = new char[bytes];
    read(file, reinterpret_cast<void*>(buf), bytes);

    data.clear();
    for(size_t i=0; i<bytes; i++) {
        data += buf[i];
    }
    delete[] buf;

    return true;
}

/** @brief Read bytes from port.
  *
  * This function checks is any device opened.
  * @param[out]  &data Link to string in which will be placed data from port.
  * @param[in]   bytes Number of bytes to read.
  */
bool Serial::readPort(std::vector<uint8_t> &data, size_t bytes)
{
    if(!isOpened()) {
        std::cerr << "File not opened yet!" << std::endl;
        return false;
    }

    if(bytesAvailable() < bytes-1) {
        std::cerr << "There are no that many bytes in file!" << std::endl;
        return false;
    }

    uint8_t *buf = new uint8_t[bytes];
    read(file, reinterpret_cast<void*>(buf), bytes);

    data.clear();
    for(size_t i=0; i<bytes; i++) {
        data.push_back(buf[i]);
    }
    delete[] buf;

    return true;
}

/** @brief Returns number of available bytes to read from buffer.
  *
  * This function returns number of available bytes to read from buffer.
  */
size_t Serial::bytesAvailable()
{
    if(!isOpened()) {
        std::cerr << "File not opened yet!" << std::endl;
        return 0;
    }

    size_t bytes = 0;
    ioctl(file, FIONREAD, &bytes);
    return bytes;
}

unsigned int copyVar(int var)
{
    unsigned int *new_var = reinterpret_cast<unsigned int*>(&var);
    return *new_var;
}
