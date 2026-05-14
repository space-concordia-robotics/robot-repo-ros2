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

        //Battery Telemetry
        if(receivedFrameID == validateFrameID(severity::SEV_STATUS, Instructions::Inst::BATTERY_TELEM)){

                batteryTelem.BatteryNum = static_cast<int>(raw_value >> 31 & 0x01);
                batteryTelem.voltage = static_cast<float>(raw_value >> 20 & 0x7FF) / VOLTAGE_MULTIPLIER; 
                batteryTelem.current = static_cast<float>(raw_value >> 6 & 0x3FFF) / CURRENT_MULTIPLIER;
                batteryTelem.temperature = static_cast<int>(raw_value & 0x3F) - 20;   
                batteryTelem.timestamp = now; 

        // Rail Telemetry
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

        // Relay Telemetry
        }
        else if(receivedFrameID == validateFrameID(severity::SEV_STATUS, Instructions::Inst::RELAY_STATUS)){
                // ASKING ELEC: Split Assignments into individual IF's for expandablility if needed. 
                relayTelem.RelayNum = static.cast<int>(raw_value & 0x01);
                relayTelem.status = static.cast<int>((raw_value >> 1) & 0x01);
        }
        //TCU Telemetry
        // TCU Temperature
        else if(receivedFrameId == validateFrameID(severity::SEV_STATUS, Instruction:Inst::TCU_TELEM)){
                float temp;
                std::memcpy(&temp ,data.data(), sizeof(float));
                tcuTelem.temperature = temp;
                tcuTeleme.timestamp = now; 
        }
        //TCU Status
        else if(receivedFrameId == validateFrameID(severity::SEV_STATUS, Instruction:Inst:TCU_STATUS)){
                tcuTelem.fanStatus = static_cast<bool>(raw_value & 0x01);
                tcuTelem.timestamp = now; 
        }
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
        std::lock_guard<std::mutex> lock(mtx);
        return railTelem.power; 
}


float BAB::getTCUStatus() const{
        std::lock_guard lock(mtx);
        return tcuTelem.fanstatus ? 1.0f : 0.0f; 
}

std::string BAB::getBMSHealth() const{
        std::lock_guard<std::mutex>lock(mtx);
        if(batteryTelem.voltage < 10.0f && batteryTelem.voltage > 0.5f){
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


bool BAB::sendKYSCommand() {
    uint16_t payload = 0x0000; 

    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_MAN_INTERVENTION, 
        static_cast<uint32_t>(Instructions::Inst::CUT_PDS_OUTPUTS),
        static_cast<uint32_t>(DeviceId::ID::BAB),
        payload
    );
}

bool BAB::cutFanPower(DeviceId::ID fanID) {
    uint16_t payload = 0x0000;

    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_CNTRL,
        static_cast<uint32_t>(Instructions::Inst::TURN_OFF_FAN),
        static_cast<uint32_t>(DeviceId::ID::BAB),
        payload
    );
}

bool BAB::CutRelayCommand(DeviceId::ID relayID) {
    uint16_t payload = (relayID == DeviceId::ID::JMSB) ? 0x000F : 0x00F0;

    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_CNTRL,
        static_cast<uint32_t>(Instructions::Inst::TURN_OFF_RELAY),
        static_cast<uint32_t>(DeviceId::ID::BAB),
        payload
    );
}

bool BAB::sendManualPowerCommands(DeviceId::ID selectRailID, bool turnOn) {
    Instructions::Inst inst = turnOn ? Instructions::Inst::COMMAND_ON : Instructions::Inst::COMMAND_OFF;
    uint16_t payload_val = (selectRailID == DeviceId::ID::ARM_EMERGENCY_INTERVENTION) ? 0x000F : 0x00F0;
   
    return build_frame.buildAddress(
        static_cast<uint32_t>(deviceType::DeviceType::BAB),
        Manufacturer::TEAM_USE,
        severity::SEV_CNTRL,
        static_cast<uint32_t>(inst),
        static_cast<uint32_t>(DeviceId::ID::BAB),
        payload_val
    );
}

    