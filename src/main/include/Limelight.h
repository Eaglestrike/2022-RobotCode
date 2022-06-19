#pragma once

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/smartdashboard/SmartDashboard.h>
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
        double getDist();
        frc::Pose2d getPose(double navx, double turretAngle);
        std::shared_ptr<nt::NetworkTable> GetNetworkTable();

        void adjustAngles(double& ax, double& ay);
        double getAdjustedX();

    private:
        void ReadPeriodicIn();
        // void adjustAngles(double& ax, double& ay);
        // double getAdjustedX();

        std::shared_ptr<nt::NetworkTable> network_table;
        std::string table_name = "limelight";

        const int PIPELINE = 0; //can change if we want non-default pipeline

        //in meters. should be pretty close?
        //TODO: confirm measurements, make separate namespace in constants
        const double TURRET_ANGLE_OFFSET = 0.0; //todo: confirm
        const double HUB_HEIGHT = 2.7178; 
        const double CAM_HEIGHT = 0.52324;
        const double CAM_ANGLE = 40; //40 degrees I think
};