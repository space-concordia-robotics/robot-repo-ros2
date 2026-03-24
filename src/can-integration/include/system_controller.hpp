#include "buildAddress.hpp"
#include "can_interface.hpp"
#include "parser.hpp"
#include "prefixes.hpp"
#include <iostream>


//This class takes care of all the system handlers, each motor has a handler function with switch cases depening on the instruction type 
//being sent. 
class SystemFramBuilder : public BuildAddress{

    public:
        

        //Start/Stop functions
        inline void StartRover(uint32_t mask); 
        inline void SystemIndicatorLight(); 
        inline void stopPayload(); 
        inline void CutBABRelays(); 

        //Jetson to motors

        //Jetson -> BAB : cuts relays to batteries, commands PDS to cut power to rails 1, 2, or 3.
        /*
            Bab also receives: Manually close a PDS rail
                               Manually close and open rails
                               Manually turn off fans
        */
        //Jetson -> Hub : Attemps to issue stop commands to ALL WHEELS SIMULTANEOUSLY. (used if wheels continue to rotate without any cmds)
        /*
        
            Hub can also receive a restart wheel command. 
        
        */
        //Jetson -> encoders : Receives commands to stop at their current position 
        /*
            Should be able to receive zeroing command from jetson
        */
        //Jetson -> Payload : Stops all current processes on the payload 

        //Jetson -> SIL : Receive RGB value, blink period, blonk off
    
        inline void getPositionalData(float position, uint8_t deviceID);
        inline void getVelocityData(float velocity, uint8_t deviceID);
        inline void haltcurrentPosition(float position, uint8_t deviceID);
 


        ~SystemFramBuilder(){std::cout << "System frame builder destructor called" << std::endl; }

    private: 


};


class SystemParser : public CANParser{ 

    //Other components (aka: bab, encoders, wheel encoders, wheel motors, etc) to jetson?? 

    /*
            The BAB will send commands to jetson such as:
            1. send status of relays
            2. send Voltage level of BMS
            3. send health 
            4. send temp 
            5. send PDS telemetry
            6. send satus of TCU

            7. Send BMS triggered cutting power
            8. send BAB triggered cutting a relay
            9. Send notice that PDS failed
            10. Send TCU report of any failure. 
    */
};