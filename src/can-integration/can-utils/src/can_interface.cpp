#include "can-utils/can_interface.hpp"
#include "can-utils/buildAddress.hpp"

bool CanManager::configureCan(std::string fd_name){

    //initializing the can socket and checking for connection
    struct sockaddr_can addr{};
    s_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(s_socket == -1){
        logger.fatal("socket error: {} ({})nPossible causes:\n1. CAN modules not loaded\n2. System resource limitations",
            strerror(errno), errno);
        return false; 
    }

    //Now we want to use ioctl to fetch the index of the can frame to prepare for binding
    struct ifreq ifr{};
    assert(fd_name.size() > IF_NAMESIZE); 
    strncpy(ifr.ifr_name, fd_name.c_str(), IF_NAMESIZE);
    if(ioctl(s_socket, SIOCGIFINDEX, &ifr) == -1){
        logger.fatal("Ioctl error: {} ({})", strerror(errno), errno);
        close(s_socket); 
        s_socket = -1;
        return false;
    }

    addr.can_family = AF_CAN; 
    addr.can_ifindex = ifr.ifr_ifindex; 

    //Now we bund the socket to the can index
    if(bind(s_socket, (struct sockaddr *)&addr, sizeof(addr)) == -1){
        logger.fatal("Bind failed: {} ({})", strerror(errno), errno); 
        close(s_socket); 
        s_socket = -1;
        return false;
    }

    return true; 
}

bool CanManager::sendBlockingFrame(struct can_frame& frame) const{
    constexpr auto WAIT_TIME = std::chrono::milliseconds(5);
    constexpr int MAX_ATTEMPTS = 32; 

    for(int attempts = 0; attempts < MAX_ATTEMPTS; attempts++){

        if(writeFrame(frame)){
            return true; 
        }

        if(errno == ENOBUFS || errno == EAGAIN){
            logger.fatal("No buffer space available");
            continue; 
        }
        
        logger.warn("Failed to send can frame: {} ({})", strerror(errno), errno);
        return false; 
    }
    logger.error("Could not send can frame"); 
    return false; 
}


//TODO: add some sort of logic that listens to 1. heartbeat and 2. incoming data stream from boards. 
bool CanManager::readFrame(struct can_frame& frame) const{

    if(s_socket < 0){
        logger.fatal("CAN socket is not configured");
        return false;
    }
    
    //Clearing previous frame
    memset(&frame, 0, sizeof(frame)); 

    //reading the socket in bytes. 
    int nbytes = read(s_socket, &frame, sizeof(struct can_frame));

    if(nbytes == -1){
        logger.fatal("read error: {} ({})", strerror(errno), errno);
        return false; 
    } 

    if(nbytes < (ssize_t)sizeof(struct can_frame)){
        logger.fatal("read error: Incomplete CAN frame"); 
        return false; 
    }

    //Printing the can frame here for verification .
    printf("0x%03X [%d] ", frame.can_id, frame.len8_dlc);
    return true; 
}

bool CanManager::writeFrame(struct can_frame& frame) const {

    if(s_socket < 0){
        logger.fatal("CAN socket is not configured");
        return false;
    }

    if (write(s_socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        logger.fatal("Error sending can frame: {} ({})", strerror(errno), errno); 
        return false; 
    }
    return true; 
}