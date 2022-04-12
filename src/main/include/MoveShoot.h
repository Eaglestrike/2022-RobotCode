#pragma once
#include <map>
#include <vector>
#include <math.h>
#include <iostream>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include "Limelight.h"
#include "SwerveDrive.h"
#include "Swerve.h"
using namespace std;

class MoveShoot  {
    public: 
        struct Settings {
            double flywheel_speed;
            double hood_angle;
            double xoff_offset;
        };

        MoveShoot(Limelight& l, Swerve& s, frc::Pose2d start);
        Settings calculate();
        //note: limelight needs to be characterized WELL for this to be useful. 
        //could do time-based thing too; like, after a certain amount of match time limelight is better than odometry
        void getPoseViaLimelight(); //will be called if limelight sees goal

    private:
        pair<int, int> distance_to_settings(double dist);
        double distance_to_time(double dist);
        double y_offset_to_dist(double y_offset);
        //these use odometry. assumes robot was zeroed from start pose
        double getXVel(double rvx, double rvy);
        double getYVel(double rvx, double rvy);

        map<double, pair<int, int>> dist_to_settings; //angle is first, speed is second
        map<double, double> dist_to_time;

        Limelight& limelight;
        Swerve& swerve;
        frc::Pose2d& startPose; //initial position and angle of robot

        //in meters. should be pretty close?
        const double HUB_HEIGHT = 2.7178; 
        const double CAM_HEIGHT = 0.52324;
        const double CAM_ANGLE = 50; //50 degrees I think

};