#include "absenc.h"
#include <iostream>

// We basically re-invented strerr()
const char* strAbsencErr(int err) {
    switch(err) {
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

ABSENC_Error_t AbsencDriver::OpenPort(const char* fileName, int& s_fd){
    errno = 0; 

    // Open TTY port via native Linux system call. Obtain its file descriptor (fd)
    s_fd = open(fileName, O_RDWR); 
    if(s_fd < 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }
    // We do need to configure the TTY port
    struct termios ttycfg; 
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0; 
    ttycfg.c_iflag = 0; 
    ttycfg.c_oflag = 0; 
    ttycfg.c_line = 0; 
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0; // Return anything read so far
    
    cfsetispeed(&ttycfg,B57600);
    cfsetospeed(&ttycfg,B57600);

    if(tcsetattr(s_fd, TCSANOW, &ttycfg) > 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }

    // All done, the resource is opened
    return no_error; 
}

// Poll slave with given slave number and serial port (fd).
// Return any errors (by value), and measurement results (by pointer).
ABSENC_Error_t AbsencDriver::PollSlave(int slvnum, ABSENC_Meas_t * meas, int s_fd){
    // Sanity check for slave numbers
     if(slvnum < 0 || slvnum > 9) {
        return ABSENC_Error_t {
            ERR_SLAVE_INVALID, 
            0, 
            __LINE__, 
        }; 
    }
    tcflush(s_fd, TCIOFLUSH); // Flush to ensure no pending TX/RX bytes at port

    // Now we construct the query packet "#0\n" where 0 represents Node ID
    char txbuf[3]; 
    txbuf[0] = '#'; 
    txbuf[1] = '0' + slvnum; 
    txbuf[2] = '\n';
    int nsend = write(s_fd, txbuf, sizeof(txbuf)); // Push to serial port
    if(nsend < 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE, 
            errno0, 
            12, 
        }; 
    }
    // tcdrain(s_fd); // Flush TX buffer? seems not needed

    // Now we try to receive the response packet.
    // First we need to search the start-of-frame symbol, which is fixed '>'. 
    char sof = 0; 
    for(int i = 0; i < 50; i++) { // Ensure SOF search always ends
        int nrecv = read(s_fd, &sof, 1); 
        std::cout << slvnum << "\t" << sof << std::endl;
        if(nrecv < 0) {
            int errno0 = errno; 
            errno = 0; 
            return ABSENC_Error_t{
                ERR_SERIAL_FAILURE, 
                errno0, 
                69, 
            }; 
        }
        for(int i = 0; i < 10000; i++);
        if(nrecv == 0) { // Timed out (encoder died)
            return ABSENC_Error_t{
                ERR_NO_RESPONSE, 
                0, 
                85, 
            }; 
        }
        if(sof == '>') break; // If it is indeed SOF, break out of the loop
        // Not SOF, maybe noise on the bus, search for another one
    }
    if(sof != '>') {
        // Not SOF and search limit exceeded. The frame is corrupted or goes very out-of-sync.
        return ABSENC_Error_t{
            ERR_FRAME_CORRUPTED, 
            0, 
            __LINE__, 
        }; 
    }

    // Response packets have a fixed format: "> X, AAAA, BBBB". X is slave number, A is position and B is status (usually zero).
    // Note that we have already received the SOF character. Ignore the \r\n that follows.
    char rxbuf[14]; 
    int nrecv = read(s_fd, rxbuf, sizeof(rxbuf)); 
    if(nrecv < 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }    
    if(nrecv < (int)sizeof(rxbuf)){
        return ABSENC_Error_t{
            ERR_FRAME_CORRUPTED, 
            0, 
            __LINE__, 
        }; 
    }

    std::cout << "Received: " << rxbuf << std::endl;

    // Debug code 
    /*
    for(int i = 0; i < nrecv; i++) {
        putchar(rxbuf[i]); 
    }
    puts(""); 
    */

    // Recap contents in rxbuf array: " X, AAAA, BBBB"
    // We ignore X, and start parsing A and B. A has an offset of 4 characters. B mmediatly follows A after 2 characters.
    uint16_t rawdata[2]; 
    int index = 4; // Start at rxbuf[4], the start of AAAA
    for(int i = 0; i < 2; i++) { // Read each hex number (total 2)
        uint16_t val = 0; 
        for(int j = 0; j < 4; j++) { // Read uint16_t hex number (4 digits)
            uint8_t nib = rxbuf[index++]; // Read one hex digit (a nibble)
            if(nib >= '0' && nib <= '9') nib = nib - '0'; 
            else if(nib >= 'A' && nib <= 'F') nib = nib - 'A' + 10; 
            else if(nib >= 'a' && nib <= 'f') nib = nib - 'a' + 10; 
            else return ABSENC_Error_t{
                ERR_FRAME_CORRUPTED, 
                0, 
                __LINE__, 
            }; 
            // Attach the nibble to the value, big-endian format
            val = (val << 4) | nib; 
        }
        // Once read, we advance pointer by 2 to account for the ", " separator
        index += 2; 
        rawdata[i] = val; // Put value into array
    }

    // Construct the measurement data storage object
    meas->slvnum = slvnum; // Slave number
    meas->status = rawdata[1]; // Status value (usually zero)
    meas->angval = ((double)(int16_t)rawdata[0]) / 65536.0 * 360.0; // Angular value, maps the uint16_t space to 360 degrees
    meas->angspd = 0.0; // Deprecated: this value is no longer provided.
    return no_error; 
}

ABSENC_Error_t AbsencDriver::ClosePort(int s_fd) {
    close(s_fd); 
    return no_error; 
}
