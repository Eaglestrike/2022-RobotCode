#pragma once

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/geometry/Pose2d.h>
#include "Constants.h"
#include <math.h>


class Limelight{
    public:
        Limelight();
        double getXOff();
        double getYOff();
        bool targetAquired();
        void setLEDMode(std::string mode);
        std::shared_ptr<nt::NetworkTable> GetNetworkTable();

    private:
        void ReadPeriodicIn();

        std::shared_ptr<nt::NetworkTable> network_table;
        std::string table_name = "limelight";

        const int PIPELINE = 0; //can change if we want non-default pipeline
};