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

enum class Status : uint8_t{
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
        char* printtatusBuffer() {
            if(!StatusBuffer_ .empty()){
                printf("%s \n", StatusBuffer_.data());
                return StatusBuffer_.data();
            }
            return nullptr; 
        }

        uint8_t configureCan(const std::vector<char> fd_name); 
        uint8_t sendFrame(struct can_frame& frame);
        uint8_t writeFrame(struct can_frame& frame); 

        void reset(int new_fd = -1){
            if(s_socket > 0){
                ::close(s_socket);
            }
            s_socket = new_fd;
            StatusBuffer_.clear(); 
        } 

    private: 
        int s_socket{-1}; 
        std::vector<char> StatusBuffer_; 
};