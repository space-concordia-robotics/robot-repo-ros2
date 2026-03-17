#pragma once 

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>
#include <cstdint>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <termios.h>
#include <net/if.h>
#include <iostream>
#include <vector>

enum Status{
    SUCCESS = 0,
    CANERROR
};

class CanManager{

    public:     
        CanManager() = default; 
        explicit CanManager(int fd_) : s_socket(fd_) {}

        CanManager(const CanManager&) = delete; 
        CanManager& operator=(const CanManager&) = delete; 

        ~CanManager(){reset();}

        int getfd() const {return s_socket;}

        static inline void printtatusBuffer() {
            printf("%s \n", s_StatusBuffer); 
        }

        uint8_t configureCan(const std::vector<char> fd_name); 
        uint8_t readFrame(struct can_frame& frame);
        uint8_t writeFrame(struct can_frame& frame); 

        void reset(int new_fd = -1){
            if(s_socket > 0){
                ::close(s_socket);
            }
            s_socket = new_fd;
            delete[] s_StatusBuffer;
        } 

    private: 
        int s_socket{-1}; 
        inline static char* s_StatusBuffer = nullptr;
};