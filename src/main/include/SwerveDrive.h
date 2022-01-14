#pragma once

#include <WheelDrive.h>
#include "math.h"
#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/PIDController.h>
#include <SwerveController.h>
#include <Odometry.h>


class SwerveDrive{

    public:
        SwerveDrive();
        void Drive (double x1, double y1, double x2, double rot, bool fieldOrient);
        void SetPID();
        void ResetEncoders();
        void TestPWM();
        void TrajectoryFollow(double fwd, double str, double rot);
        void SetDriveControllerPID();
        void UpdateOdometry(double ROT, double theta);
        void ResetOdometry();


    private:
        //Width and Length of the distance between wheel axle
        double m_W = 0.394;
        double m_L = 0.394;

        double m_temp;

        WheelDrive m_backRight{18, 17, 6, 7, 13}; //PWM 3
        WheelDrive m_backLeft{16, 15, 4, 5, 12}; //PWM 2
        WheelDrive m_frontRight{14, 13, 2, 3, 11}; //PWM 1
        WheelDrive m_frontLeft{12, 11, 0, 1, 10}; //PWm 0

        SwerveController m_swerveController;

        Odometry m_odometry;
};  