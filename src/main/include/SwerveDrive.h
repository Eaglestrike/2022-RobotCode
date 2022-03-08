#pragma once

#include <WheelDrive.h>
#include "math.h"
#include <frc/controller/PIDController.h>
#include <SwerveController.h>
#include <Odometry.h>
#include <Trajectory.h>
#include <Constants.h>
#include <AHRS.h>


class SwerveDrive{

    public:
        SwerveDrive();
        void Drive (double x1, double y1, double x2, double rot, bool fieldOrient);
        void SetPID();
        void TrajectoryFollow(double rot, size_t waypointIndex = 9999);
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
        void GenerateTrajectory_2();

        void debug(AHRS &navx);


    private:
        double m_W = DriveConstants::Width;
        double m_L = DriveConstants::Length;

        double m_temp;

        double m_FL_Offset = DriveConstants::FLOFF;
        double m_FR_Offset = DriveConstants::FROFF;
        double m_BL_Offset = DriveConstants::BLOFF;
        double m_BR_Offset = DriveConstants::BROFF;

        WheelDrive m_backRight{DriveConstants::BRanglePort,
            DriveConstants::BRspeedPort, DriveConstants::BRencoder};
        WheelDrive m_backLeft{DriveConstants::BLanglePort,
            DriveConstants::BLspeedPort, DriveConstants::BLencoder};
        WheelDrive m_frontRight{DriveConstants::FRanglePort,
            DriveConstants::FRspeedPort, DriveConstants::FRencoder};
        WheelDrive m_frontLeft{DriveConstants::FLanglePort,
            DriveConstants::FLspeedPort, DriveConstants::FLencoder};

        SwerveController m_swerveController;

        Odometry m_odometry;

        Trajectory m_trajectory_1;
        Trajectory m_trajectory_2;

        AHRS *gyro;
};  