#include "battery_board.cpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <diagnostic_updater/diagnostic_updater.hpp>


using namespace std::chrono_literals;
using namespace diagnostic_msgs::msg; 


    class ProduceDiagnostics : public rclcpp::Node{

        public: 

            ProduceDiagnostics(): Node("Produce_Diagnostic_node"), updater_(this) {

                updater_.add("Power Status: ", this, &ProduceDiagnostics::checkBatteryVoltage);
                updater_.add("Power Status: ", this, &ProduceDiagnostics::checkRailVoltage);

                updater_.add("Current Status: ", this, &ProduceDiagnostics::checkCurrent);
                updater_.add("Temperature Status: ", this, &ProduceDiagnostics::checkTemperature);
                updater_.add("Module Status: ", this, &ProduceDiagnostics::checkStatus);

                updater_.setPeriod(10); 

            }

        private:   


            void checkBatteryVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {

                const float batteryVoltage = diagnostics_ptr->getBatteryVoltageLevel(); 

                stat.add("Battery Voltage Level (V): ", batteryVoltage); 

                if(batteryVoltage >= 10 && batteryVoltage <= 14.8){
                    stat.summary(DiagnosticStatus::OK, "Batter levels OK"); 
                }else if(batteryVoltage > 14.8){
                    stat.summary(DiagnosticStatus::ERROR, "Battery voltage exceeding criticality, shut rover off immediately"); 
                }else if(batteryVoltage <= 10){
                    stat.summary(DiagnosticStatus::WARN, "Battery voltage Low, replace batteries"); 
                }

            }

            void checkRailVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {

            }
            
            void checkCurrent(diagnostic_updater::DiagnosticStatusWrapper& stat){


            }
            
            void checkPower(diagnostic_updater::DiagnosticStatusWrapper& stat){
                
            }

            void checkTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat){

            } 

            void checkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat){

            } 
        
        
        diagnostic_updater::Updater updater_ ;
        rclcpp::TimerBase::SharedPtr clock_; 
        std::shared_ptr<BAB> diagnostics_ptr = std::make_shared<BAB>(); 


};
