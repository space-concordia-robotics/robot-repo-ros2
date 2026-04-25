#include "can-utils/can_interface.hpp"
#include "can-utils/prefixes.hpp"
#include <mutex> 
#include <string> 
#include <chrono>
#include <vector> 



//TODO (Michael): Figure out the command value associated with each desired telemetry point. 


/*
    * @brief Struct storing important telemetry updates to be printed on the terminal.
*/
struct telemetryUpdates{
    float voltage;
    float temperature; 
    float current; 
    std::string RelayStatus; 
    std::string BABStatus; 
    std::string BMSHealth; 
    std::string TCUStatus; 
    std::chrono::steady_clock::time_point timestamp; 
};

 /*
*  @class BAB
*  @brief Battery Arbiter Board class 
*  
*   This class provides methods to obtain required telemetry from the BAB which will then be displayed on the terminal as data input for users. 
*
*/

class BAB{

    public: 
    
        /*
        * @brief initiallizes the can controller to establish a CAN connection in order to obtain telemetry. 
        *
        * @details This constructor attempts to initialize the CAN bus connection. If it fails, it will throw
        * an exception with a detailed error message that includes possible causes and suggested solutions.
        *  
        */

        BAB(rclcpp::Logger& logger, can_util::CANController& can_controller_); 


        void handleFrames(const DeviceId::ID id, const std::vector<uint8_t>& data); 



        /*
        *  @brief Gets the current battery voltage level. 
        *  @param: None  
        *  @return return a voltage value in Volts. 
        * 
        */
        float getVoltageLevel() const; 

        /*
        *  @brief Gets the current running through the battery.
        *  @param: None  
        *  @return return a voltage value in Volts. 
        * 
        */
        float getCurrentLevel() const; 





        /*
        *  @brief Aquires the BMS temperature 
        *  @param: None 
        * 
        */
        float getBMSTemp() const;




        /*
        *  @brief Gets the power distribution system telemetry 
        *  @param: None
        * 
        */
        float getPDSTelemetry() const; 



        /*
        *  @brief Sends report to user if TCU is no longer operational
        *  @param: None
        * 
        */
        float getTCUStatus() const; 


        /*
        *  @brief 
        *  @param: None 
        *  @return: Returns a string indicating the status of the BMS 
        */
        std::string getBMSHealth() const; 


        /*
        *  @brief Gets the status of the power relays for the arm, BAB and wheel motor power rails. 
        *  @param: None
        *   
        */
        std::string getRelayStatus() const; 


        /*
        *  @brief Reports when the BAB has decided to cut the relays to the batteries or 
        *   when the BAB reports that the PDS has decided to cut a power rail. 
        *
        *  @details
        *   E00: Both batteries have reached their lower thresholds.
        *   E01: Unexpected Failure. Check fuses.
        *   E02: The batteries are overheating.
        *   E11: BlueBus power rail is shut off.
        *   E12: Wheel power rail is shut off.
        *   E13: Arm power rail is shut off.
        * 
        * @param: None
        *  
        */
        std::string getBABStatus() const; 


        /*
        *  @brief Sends command to cut ALL rails on the PDS
        *  @param: None. 
        *  @return: True for success 
        */
        bool sendKYSCommand(); 



        /*
        *  @brief Sends command to cut fan power
        *  @param: DeviceId::ID fanid
        *  @details --> Commands get sent to TCU board. 
        *  @return: True for success
        */
        bool cutFanPower(DeviceId::ID fanID); 


        /*
        *  @brief Sends command to cut ALL relay power
        *  @param: DeviceId::ID relay id 
        *  @return: True for success
        */
        bool CutRelayCommand(DeviceId::ID relayID); 

        /*
        *  @brief Send commands to manually cut power to rails 1, 2 or 3 or open them again. 
        *
        *    1: BlueBus Rail
        *    2: Arm Rail
        *    3: Wheels Rail
        *  @param: DeviceId::ID railID 
        *  @return: True for success
        */
        bool sendManualPowerCommands(DeviceId::ID selectRailID); 

    
    private:

        mutable std::mutex mtx; 
        ros2_fmt_logger::Logger logger; 
        can_util::CANController& can_controller; 


}; 