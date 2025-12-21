#include "arm_hardware/absenc.h"

// We basically re-invented strerr()
const char* strAbsencErr(int err)
{
    switch (err) {
        case NO_ERROR:
            return "No error occurred";
        case ERR_SERIAL_FAILURE:
            return "Serial failure";
        case ERR_SLAVE_INVALID:
            return "Slave invalid";
        case ERR_NO_RESPONSE:
            return "No response";
        case ERR_FRAME_CORRUPTED:
            return "Frame corrupted";
        default:
            return "Unknown code";
    }
}

ABSENC_Error_t AbsencDriver::OpenPort(const char* fileName, int& s_fd)
{
    errno = 0;

    // Open TTY port via native Linux system call. Obtain its file descriptor (fd)
    s_fd = open(fileName, O_RDWR);
    if (s_fd < 0) {
        const int errno0 = errno;
        errno = 0;
        return ABSENC_Error_t{ERR_SERIAL_FAILURE, errno0, __LINE__};
    }

    // Configure the TTY port
    struct termios ttycfg;
    memset(&ttycfg, 0, sizeof(ttycfg));
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0;
    ttycfg.c_iflag = 0;
    ttycfg.c_oflag = 0;
    ttycfg.c_line = 0;
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0;  // Return anything read so far

    cfsetispeed(&ttycfg, B57600);
    cfsetospeed(&ttycfg, B57600);

    if (tcsetattr(s_fd, TCSANOW, &ttycfg) > 0) {
        const int errno0 = errno;
        errno = 0;
        return ABSENC_Error_t{ERR_SERIAL_FAILURE, errno0, __LINE__};
    }

    // All done, the resource is opened
    return no_error;
}

ABSENC_Error_t AbsencDriver::PollSlave(int slvnum, ABSENC_Meas_t* meas, int s_fd)
{
    // Sanity check for slave numbers
    if (slvnum < 0 || slvnum > 9) {
        return ABSENC_Error_t{ERR_SLAVE_INVALID, 0, __LINE__};
    }

    tcflush(s_fd, TCIOFLUSH); // Flush to ensure no pending TX/RX bytes at port

    // Construct the query packet "#0\n" where 0 represents Node ID
    char txbuf[3];
    txbuf[0] = '#';
    txbuf[1] = static_cast<char>('0' + slvnum);
    txbuf[2] = '\n';

    const int nsend = write(s_fd, txbuf, sizeof(txbuf));
    if (nsend < 0) {
        const int errno0 = errno;
        errno = 0;
        return ABSENC_Error_t{ERR_SERIAL_FAILURE, errno0, __LINE__};
    }

    // Search start-of-frame symbol, which is fixed '>'
    char sof = 0;
    for (int i = 0; i < 50; i++) {
        const int nrecv = read(s_fd, &sof, 1);
        if (nrecv < 0) {
            const int errno0 = errno;
            errno = 0;
            return ABSENC_Error_t{ERR_SERIAL_FAILURE, errno0, __LINE__};
        }
        if (nrecv == 0) {
            // Timed out (encoder died)
            return ABSENC_Error_t{ERR_NO_RESPONSE, 0, __LINE__};
        }
        if (sof == '>') {
            break;
        }
        // Not SOF, maybe noise on the bus, search for another one
    }

    if (sof != '>') {
        // Not SOF and search limit exceeded. The frame is corrupted or out-of-sync.
        return ABSENC_Error_t{ERR_FRAME_CORRUPTED, 0, __LINE__};
    }

    // Read the response line until '\n' or a small cap.
    // Expected format (example): >0 0000 123.456 7.890\n
    char rxbuf[128];
    size_t n = 0;
    rxbuf[n++] = sof;

    while (n < sizeof(rxbuf) - 1) {
        char c = 0;
        const int nrecv = read(s_fd, &c, 1);
        if (nrecv < 0) {
            const int errno0 = errno;
            errno = 0;
            return ABSENC_Error_t{ERR_SERIAL_FAILURE, errno0, __LINE__};
        }
        if (nrecv == 0) {
            return ABSENC_Error_t{ERR_NO_RESPONSE, 0, __LINE__};
        }
        rxbuf[n++] = c;
        if (c == '\n') {
            break;
        }
    }

    rxbuf[n] = '\0';

    // Parse: ">" slvnum status angval angspd
    int slv = 0;
    unsigned int status = 0;
    double angval = 0.0;
    double angspd = 0.0;

    // Accept both hex and decimal status; in practice it's hex in logs.
    const int parsed = sscanf(rxbuf, ">%d %x %lf %lf", &slv, &status, &angval, &angspd);
    if (parsed < 4) {
        return ABSENC_Error_t{ERR_FRAME_CORRUPTED, 0, __LINE__};
    }

    meas->slvnum = static_cast<uint8_t>(slv);
    meas->status = static_cast<uint16_t>(status);
    meas->angval = angval;
    meas->angspd = angspd;

    return no_error;
}

ABSENC_Error_t AbsencDriver::ClosePort(int s_fd)
{
    if (s_fd >= 0) {
        close(s_fd);
    }
    return no_error;
}
