#include "buildAddress.hpp"
#include "can_interface.hpp"
#include "parser.hpp"
#include "prefixes.hpp"
#include <iostream>


class SystemFramBuilder : public BuildAddress{

    public:
        
        //Jetson to motors
        inline void getPositionalData(float position, uint8_t deviceID);
        inline void getVelocityData(float velocity, uint8_t deviceID);
        inline void haltcurrentPosition(float position, uint8_t deviceID);

        //Start/Stop functions
        inline void StartRover(uint32_t mask); 
        inline void SystemIndicatorLight(); 
        inline void stopPayload(); 
        inline void CutBABRelays(); 

 


        ~SystemFramBuilder(){std::cout << "System frame builder destructor called" << std::endl; }

    private: 


};


class SystemParser : public CANParser{ 

    //Other components (aka: bab, encoders, wheel encoders, wheel motors, etc) to jetson?? 
};