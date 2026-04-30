#include "battery_board.hpp"


BAB::BAB(rclcpp::Logger& logger, 
         can_util::CANController& can_controller_, 
         buildAddress::BuildAddress& build_frame, 
         uint32_t deviceID)

        : logger(logger.get_child("battery_arbiter_board")), 
          can_controller(can_controller_), 
          build_frame(build_frame),
          deviceId(deviceID){
                
                frame_callback = can_controller.registerFrameCallback([this](const uint32_t frameid, const std::vector<uint8_t>& data){
                        handleFrames(frameid, data);
                });
        }

uint32_t BAB::validateFrameID(uint32_t sev, Instructions::Inst cmd) const{

        return buildAddress::BuildAddress::buildCANID(static_cast<uint32_t>(deviceType::DeviceType::BAB),
                                                      Manufacturer::TEAM_USE, 
                                                      sev,
                                                      static_cast<uint32_t>(cmd),
                                                      static_cast<uint32_t>(DeviceId::ID::BAB)); 
} 

//TODO (Michael): Figure out what the id of each telemetry command should be and then decode it here.
void BAB::handleFrames(const uint32_t id, const std::vector<uint8_t>& data){

        const uint32_t receivedFrameID = id; 
        uint64_t raw_value = 0; 

        for(auto i = 0u; i < data.size(); ++i){
                raw_value |= static_cast<uint64_t>(data[i]) << (8*i);   
        }

        const auto now = std::chrono::steady_clock::now(); 

        std::lock_guard(mtx); 

        if(receivedFrameID == validateFrameID(severity::SEV_STATUS, Instructions::Inst::BATTERY_TELEM)){

                batteryTelem.voltage = 0.0; 
        }


}


float BAB::getVoltageLevel() const{

}


float BAB::getCurrentLevel() const{

} 


float BAB::getBMSTemp() const{

}


float BAB::getPDSTelemetry() const{

}


float BAB::getTCUStatus() const{

}

std::string BAB::getBMSHealth() const{

}

std::string BAB::getRelayStatus() const{

}


std::string BAB::getBABStatus() const{

}


bool BAB::sendKYSCommand(){

}

bool BAB::cutFanPower(DeviceId::ID fanID){

}


bool BAB::CutRelayCommand(DeviceId::ID relayID){

}


bool BAB::sendManualPowerCommands(DeviceId::ID selectRailID){
    
} 

    