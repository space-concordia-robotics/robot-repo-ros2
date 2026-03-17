#include "can_interface.hpp"
#include "buildAddress.hpp"


uint8_t CanManager::configureCan(std::vector<char> fd_name){

    s_StatusBuffer = new char[1000];
    //initializing the can socket and checking for connection
    struct sockaddr_can addr{};
    s_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(s_socket == -1){
        sprintf(s_StatusBuffer, "Socket error, %s (%i)\n", strerror(errno), errno); 
        return Status::CANERROR; 
    }

    //Now we want to use ioctl to fetch the index of the can frame to prepare for binding
    struct ifreq ifr{};
    strcpy(ifr.ifr_name, fd_name.data());
    if(ioctl(s_socket, SIOCGIFINDEX, &ifr) == -1){
        sprintf(s_StatusBuffer,"Iocl error : %s (%i)\n", strerror(errno),errno);
        return Status::CANERROR;
    }

    addr.can_family = AF_CAN; 
    addr.can_ifindex = ifr.ifr_ifindex; 

    //Now we bund the socket to the can index
    if(bind(s_socket, (struct sockaddr *)&addr, sizeof(addr)) == -1){
        sprintf(s_StatusBuffer, "Bind error : %s (%i)\n", strerror(errno),errno);
        return Status::CANERROR;
    }

    return SUCCESS; 
}