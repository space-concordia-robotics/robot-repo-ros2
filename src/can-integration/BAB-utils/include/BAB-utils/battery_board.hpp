#include "can-utils"
#include <mutex> 


class BAB{

    public: 

        void getRelayStatus(); 
        float getVoltageLevel(); 
        void getBMSHealth(); 
        float getBMSTemp();
        float getPDSTelemetry(); 
        float getTCUStatus(); 
        float getBABStatus(); 

    
    private:





}; 