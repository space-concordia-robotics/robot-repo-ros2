#include "arm_hardware/absenc.hpp"

#include <cerrno>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

// We basically re-invented strerr()
const char* to_string(const AbsencErrorCause cause) {
    switch (cause) {
    case AbsencErrorCause::NONE:
        return "No error occurred";
    case AbsencErrorCause::SERIAL_FAILURE:
        return "Serial failure";
    case AbsencErrorCause::SLAVE_INVALID:
        return "Slave invalid";
    case AbsencErrorCause::NO_RESPONSE:
        return "No response";
    case AbsencErrorCause::FRAME_CORRUPTED:
        return "Frame corrupted";
    default:
        return "Unknown code";
    }
}

AbsencError AbsencDriver::openPort(const char* path, int& s_fd) {
    // Open TTY port via native Linux system call. Obtain its file descriptor (fd)
    s_fd = open(path, O_RDWR);
    if (s_fd < 0) {
        return AbsencError{
            .error = AbsencErrorCause::SERIAL_FAILURE,
            .cause = errno,
            .line  = __LINE__,
        };
    }
    // We do need to configure the TTY port
    termios ttycfg = {};
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0;
    ttycfg.c_iflag = 0;
    ttycfg.c_oflag = 0;
    ttycfg.c_line = 0;
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0; // Return anything read so far

    cfsetispeed(&ttycfg,B57600);
    cfsetospeed(&ttycfg,B57600);

    if (tcsetattr(s_fd, TCSANOW, &ttycfg) > 0) {
        return AbsencError{
            .error = AbsencErrorCause::SERIAL_FAILURE,
            .cause = errno,
            .line  = __LINE__,
        };
    }

    // All done, the resource is opened
    return NO_ERROR;
}

// Poll slave with given slave number and serial port (fd).
// Return any errors (by value), and measurement results (by pointer).
AbsencError AbsencDriver::pollSlave(const int slvnum, AbsencMeasurement* meas, const int s_fd) {
    // Sanity check for slave numbers
    if (slvnum < 0 || slvnum > 9) {
        return AbsencError{
            .error = AbsencErrorCause::SLAVE_INVALID,
            .cause = 0,
            .line  = __LINE__,
        };
    }
    tcflush(s_fd, TCIOFLUSH); // Flush to ensure no pending TX/RX bytes at port

    // Now we construct the query packet "#0\n" where 0 represents Node ID
    const std::array<char, 3> txbuf = {'#', static_cast<char>('0' + slvnum), '\n'};
    if (const int nsend = write(s_fd, txbuf.data(), txbuf.size()); nsend < 0) {
        return AbsencError{
            .error = AbsencErrorCause::SERIAL_FAILURE,
            .cause = errno,
            .line  = __LINE__,
        };
    }
    // tcdrain(s_fd); // Flush TX buffer? seems not needed

    // Now we try to receive the response packet.
    // First we need to search the start-of-frame symbol, which is fixed '>'.
    char start_of_frame = 0;
    for (int i = 0; i < 50; i++) {
        // Ensure SOF search always ends
        const int received = read(s_fd, &start_of_frame, 1);
        if (received < 0) {
            return AbsencError{
                .error = AbsencErrorCause::SERIAL_FAILURE,
                .cause = errno,
                .line  = __LINE__,
            };
        }
        if (received == 0) {
            // Timed out (encoder died)
            return AbsencError{
                .error = AbsencErrorCause::NO_RESPONSE,
                .cause = 0,
                .line  = __LINE__,
            };
        }
        if (start_of_frame == '>')
            break; // If it is indeed SOF, break out of the loop
        // Not SOF, maybe noise on the bus, search for another one
    }

    if (start_of_frame != '>') {
        // Not SOF and search limit exceeded. The frame is corrupted or goes very out-of-sync.
        return AbsencError{
            .error = AbsencErrorCause::FRAME_CORRUPTED,
            .cause = 0,
            .line  = __LINE__,
        };
    }

    // Response packets have a fixed format: "> X, AAAA, BBBB". X is slave number, A is position and B is status (usually zero).
    // Note that we have already received the SOF character. Ignore the \r\n that follows.
    std::array<char, 14> rxbuf = {};
    const int received = read(s_fd, rxbuf.data(), rxbuf.size());
    if (received < 0) {
        return AbsencError{
            .error = AbsencErrorCause::SERIAL_FAILURE,
            .cause = errno,
            .line  = __LINE__,
        };
    }
    if (std::cmp_less(received, sizeof(rxbuf))) {
        return AbsencError{
            .error = AbsencErrorCause::FRAME_CORRUPTED,
            .cause = 0,
            .line  = __LINE__,
        };
    }


    // Debug code
    /*
    for(int i = 0; i < nrecv; i++) {
        putchar(rxbuf[i]);
    }
    puts("");
    */

    // Recap contents in rxbuf array: " X, AAAA, BBBB"
    // We ignore X, and start parsing A and B. A has an offset of 4 characters. B mmediatly follows A after 2 characters.
    std::array<uint16_t, 2> rawdata = {};
    int index = 4; // Start at rxbuf[4], the start of AAAA
    for (int i = 0; i < 2; i++) {
        // Read each hex number (total 2)
        uint16_t value = 0;
        for (int j = 0; j < 4; j++) {
            // Read uint16_t hex number (4 digits)
            uint8_t nibble = rxbuf.at(index++); // Read one hex digit (a nibble)
            if (nibble >= '0' && nibble <= '9')
                nibble = nibble - '0';
            else if (nibble >= 'A' && nibble <= 'F')
                nibble = nibble - 'A' + 10;
            else if (nibble >= 'a' && nibble <= 'f')
                nibble = nibble - 'a' + 10;
            else
                return AbsencError{
                    .error = AbsencErrorCause::FRAME_CORRUPTED,
                    .cause = 0,
                    .line  = __LINE__,
                };
            // Attach the nibble to the value, big-endian format
            value = value << 4 | nibble;
        }
        // Once read, we advance pointer by 2 to account for the ", " separator
        index += 2;
        rawdata.at(i) = value; // Put value into array
    }

    // Construct the measurement data storage object
    meas->slvnum = slvnum; // Slave number
    meas->status = rawdata.at(1); // Status value (usually zero)
    meas->angval = static_cast<double>(static_cast<int16_t>(rawdata.at(0))) / 65536.0 * 360.0; // Angular value, maps the uint16_t space to 360 degrees
    meas->angspd = 0.0; // Deprecated: this value is no longer provided.
    return NO_ERROR;
}

AbsencError AbsencDriver::closePort(const int s_fd) {
    close(s_fd);
    return NO_ERROR;
}
