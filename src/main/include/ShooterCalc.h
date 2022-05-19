//when you add moving while shooting the shooter calculations get to be hefty, so I thought I'd make it a separate class

#pragma once
#include <map>
#include <vector>
#include <math.h>
#include <iostream>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include "Limelight.h"
#include "Swerve.h"
using namespace std;

class ShooterCalc  {
    public: 
        struct Settings { 
            double flywheel_speed;
            double hood_angle;
            double xoff_offset;
        };

        ShooterCalc(Limelight& l, Swerve& s);
        Settings calculate();
        //should I move this to a different class? Swerve or Limelight?
        void getPoseViaLimelight(); //TODO: write

    private:
        pair<int, int> distance_to_settings(double dist);
        double distance_to_time(double dist);
        double y_offset_to_dist(double y_offset);
        double getXVel(double rvx, double rvy, frc::Pose2d rcoords);
        double getYVel(double rvx, double rvy, frc::Pose2d rcoords);

        map<double, pair<int, int>> dist_to_settings; //angle is first, speed is second
        map<double, double> dist_to_time;

        Limelight& limelight;
        Swerve& swerve;

        //in meters. should be pretty close?
        //TODO: confirm measurements
        const double HUB_HEIGHT = 2.7178; 
        const double CAM_HEIGHT = 0.52324;
        const double CAM_ANGLE = 50; //50 degrees I think

};