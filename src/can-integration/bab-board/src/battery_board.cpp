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
        const DecodedFrame decoded = CANParser::parse(id, data); 
        const uint32_t railInst = decoded.instruction; 
        uint64_t raw_value = 0; 

        for(auto i = 0u; i < data.size(); ++i){
                raw_value |= static_cast<uint64_t>(data[i]) << (8*i);   
        }

        const auto now = std::chrono::steady_clock::now(); 

        std::lock_guard lock(mtx); 

        if(receivedFrameID == validateFrameID(severity::SEV_STATUS, Instructions::Inst::BATTERY_TELEM)){

                batteryTelem.BatteryNum = static_cast<int>(raw_value >> 31 & 0x01);
                batteryTelem.voltage = static_cast<float>(raw_value >> 20 & 0x7FF) / VOLTAGE_MULTIPLIER; 
                batteryTelem.current = static_cast<float>(raw_value >> 6 & 0x3FFF) / CURRENT_MULTIPLIER;
                batteryTelem.temperature = static_cast<int>(raw_value & 0x3F) - 20;   
                batteryTelem.timestamp = now; 

        }else if(receivedFrameID == validateFrameID(severity::SEV_STATUS, Instructions::Inst::RAIL_TELEM)){

                railTelem.RailNum = static_cast<int>(raw_value >> 43 & 0x01); 

                if(railInst == 0x04){

                        const uint8_t railCommand = static_cast<uint8_t>(raw_value & 0xFFFF); 
                        if(railCommand == 0x000F){
                                
                                railTelem.status = false; 

                        }else if(railCommand == 0x00F0){

                                railTelem.status = true; 
                        }
                }

                railTelem.voltage = static_cast<float>(raw_value >> 30 & 0x3FFFFFFF) / VOLTAGE_MULTIPLIER; 
                railTelem.power = static_cast<float>(raw_value >> 16 & 0xFFFF) / POWER_MULTIPLIER;


        }else if(receivedFrameID == validateFrameID(severity::SEV_STATUS, Instructions::Inst::RELAY_STATUS)){
                // ASKING ELEC: Split Assignments into individual IF's for expandablility if needed. 
                relayTelem.RelayNum = static.cast<int>(raw_value & 0x01);
                relayTelem.status = static.cast<int>((raw_value >> 1) & 0x01);
        }

        // TODO: The Remaining GETTTERS  - Luca

}


float BAB::getBatteryVoltageLevel() const{
        std::lock_guard<std::mutex>lock(mtx);
        return BatteryTelem.voltage;
}


float BAB::getBatteryCurrentLevel() const{
        std::lock_guard<std::mutex>lock(mtx);
        return BatteryTelem.current;
} 


float BAB::getBatteryTemp() const{
        std::lock_guard<std::mutex>lock(mtx);
        return BatteryTelem.temperature; 
}


float BAB::getPDSTelemetry() const{

}


float BAB::getTCUStatus() const{

}

std::string BAB::getBMSHealth() const{
        std::lock_guard<std::mutex>lock(mtx);
        return BatteryTelem.voltage < 10.0f && BatteryTelem.voltage > 0.5f{
                return "CRITICAL LOW VOLTAGE"; 
        }
        return "NORMAL VOLTAGE (HEALTHY)";
}

std::string BAB::getRelayStatus() const{
        std::lock_guard<std::mutex>lock(mtx);
        return RealayTelem.status ? "Relay Closed (ON)" : "Relay OPEN (OFF)"; 
}

std::string BAB::getBABStatus() const{
        std::lock_guard lock(mtx);
        retunr "NORMAL";
}


bool BAB::sendKYSCommand(){
        uint32_t id = validateFrameID(0, Instructions::Inst::CUT_PDS_OUTOUTS);
        std::vector<uint8_t data = {0x00, 0x00};;
        return can_controller.sendFrame(id,data); 
}

bool BAB::cutFanPower(DeviceId::ID fanID){
        uint32_t id = validateFrameID(severity::SEV_CNTRL. Instructions::Inst::TURN_OFF_FAN);
        std::vector<uint8_t> data(0,0);
        return can_controller.sendFrame(id,data);
}


bool BAB::CutRelayCommand(DeviceId::ID relayID){
        uint32__t id = validateFrameID(severity::SEV_CNTRL, Instructions::Inst::TURN_OFF_RELAY);
        std::vector<uint8_t>data(0,0);
        data[0] = (relayID == DeviceId::ID::JMSB) ? 0x0F : 0xF0; 
        return can_controller.sendFrame(id,data);
}


bool BAB::sendManualPowerCommands(DeviceId::ID selectRailID){

} 

    