/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Limelight header file

  Initializes appriopriate functions and network tables etc.. 
*/


#pragma once

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Constants.h"
#include <math.h>


class Limelight{
    public:
        Limelight();
        double calculateDistance();
        double getXOff();
        double getYOff();
        bool targetAquired();
        void setLEDMode(std::string mode);
        std::shared_ptr<nt::NetworkTable> GetNetworkTable();

    private:
        void ReadPeriodicIn();

        std::shared_ptr<nt::NetworkTable> network_table;
        std::string table_name = "limelight";

        const int PIPELINE = 0;
};