#pragma once

#include <WheelDrive.h>
#include "math.h"
#include <frc/controller/PIDController.h>
#include <SwerveController.h>
#include <Odometry.h>
#include <Trajectory.h>


class SwerveDrive{
    public:
        SwerveDrive();
        void Drive (double x1, double y1, double x2, double rot, bool fieldOrient);
        void SetPID();
        void TrajectoryFollow(double rot, bool vel);
        void SetDriveControllerPID();
        void SetDriveControllerROTPID();
        void UpdateOdometry(double theta);
        void ResetOdometry();
        void Initialize();
        void ResetEncoders();
        double GetYPosition();
        double GetXPostion();
        double GetYSpeed();
        double GetXSpeed();
        double calcYawStraight(double targetAngle, double currentAngle);
        void GenerateTrajectory_1();
        void GeneratePath();

        void debug();


    private:
        double m_W = 0.394;
        double m_L = 0.394;

        double m_temp;

        double m_FL_Offset = 0.43;
        double m_FR_Offset = 0.695;
        double m_BL_Offset = 0.78;
        double m_BR_Offset = 0.19;

        WheelDrive m_backRight{18, 17, 6, 7, 13}; //PWM 3
        WheelDrive m_backLeft{16, 15, 4, 5, 12}; //PWM 2
        WheelDrive m_frontRight{14, 13, 2, 3, 11}; //PWM 1
        WheelDrive m_frontLeft{12, 11, 0, 1, 10}; //PWm 0

        SwerveController m_swerveController;

        Odometry m_odometry;

        Trajectory m_trajectory_1;
        Trajectory m_trajectory_2;
};  