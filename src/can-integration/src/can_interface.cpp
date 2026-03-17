#include "can_interface.hpp"
#include "buildAddress.hpp"


uint8_t CanManager::configureCan(std::vector<char> fd_name){

    StatusBuffer_.resize(1024);
    struct sockaddr_can addr{};
    s_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

}