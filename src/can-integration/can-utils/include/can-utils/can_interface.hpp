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
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <assert.h>
#include <chrono>

class CanManager{

    public:  
    
        CanManager(const std::string& interface_name, rclcpp::Logger logger); 

        
        bool configureCan(std::string fd_name); 
        
        bool sendBlockingFrame(struct can_frame& frame) const; 
        
        bool writeFrame(struct can_frame& frame) const; 
        bool readFrame(struct can_frame& frame) const; 
        
        ~CanManager(){
            std::cout<< "CanManager destructor called" << std::endl; 
            if(s_socket > 0){
                close(s_socket);
            }
        }
        
    private: 
        std::string interface_name; 
        ros2_fmt_logger::Logger logger; 
        int s_socket = -1; 
};