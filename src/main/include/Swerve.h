#pragma once
#include <string>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <Constants.h>
#include "DataLogger.h"
#include <frc/MathUtil.h>
#include <AHRS.h>
#include <vector>

class Swerve {
    public:
        Swerve(AHRS * nx, DataLogger * logger);
        void Periodic(units::meters_per_second_t joy_x, units::meters_per_second_t joy_y, 
        units::radians_per_second_t joy_theta, units::degree_t navx_yaw);
        void DisabledPeriodic(wpi::array<frc::SwerveModuleState, 4> * moduleStates);

        frc::ChassisSpeeds getSpeeds();
        frc::Pose2d getPose() { return odometry->GetPose(); }

        void initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose);
        void updateOdometry(frc::Rotation2d robotAngle, frc::Pose2d robotPose) { odometry->ResetPosition(robotPose, robotAngle); }

    std::vector<TalonFX *> getTalons(); //for simulation

    private:

    DataLogger * m_logger{nullptr};

    AHRS * m_navx;

    double ticksToDeg(double ticks) { 
        return frc::InputModulus(ticks, -1024.0, 1024.0) / 1024.0 * 180.0; //SHOULD result in being between -180 and 180
    }

    void SetPID();

    double aPrevError_, aIntegralError_, dPrevError_, dIntegralError_;

    //converts raw talon velocity to meters per second
    //raw velocity units are ticks per 100ms
    units::meters_per_second_t talonVelToMps(double vel) {
        double wheel_radius = 0.05;
        double meters_per_rev = wheel_radius*2*M_PI; //wheel circumberence
        double ticks_per_rev = 2048;
        return units::meters_per_second_t{vel / 0.1 * (meters_per_rev / ticks_per_rev)};

    }

        // Swerve module system
    // SwerveDrive m_swerve;
    // Locations for the swerve drive modules relative to the robot center.
    frc::Translation2d m_frontLeftLocation{0.3683_m, 0.3683_m};
    frc::Translation2d m_frontRightLocation{0.3683_m, -0.3683_m};
    frc::Translation2d m_backLeftLocation{-0.3683_m, 0.3683_m};
    frc::Translation2d m_backRightLocation{-0.3683_m, -0.3683_m};

    // Creating my kinematics object using the module locations.
    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, m_frontRightLocation,
        m_backLeftLocation, m_backRightLocation
    };

    // Underlying swerve module sensors/actuators
    WPI_TalonFX  m_fl_angleMotor{DriveConstants::FLanglePort, "Drivebase"};
    WPI_TalonFX  m_fl_speedMotor{DriveConstants::FLspeedPort, "Drivebase"};
    WPI_CANCoder m_fl_canCoder  {DriveConstants::FLencoder,   "Drivebase"};
    WPI_TalonFX  m_fr_angleMotor{DriveConstants::FRanglePort, "Drivebase"};
    WPI_TalonFX  m_fr_speedMotor{DriveConstants::FRspeedPort, "Drivebase"};
    WPI_CANCoder m_fr_canCoder  {DriveConstants::FRencoder,   "Drivebase"};
    WPI_TalonFX  m_rl_angleMotor{DriveConstants::BLanglePort, "Drivebase"};
    WPI_TalonFX  m_rl_speedMotor{DriveConstants::BLspeedPort, "Drivebase"};
    WPI_CANCoder m_rl_canCoder  {DriveConstants::BLencoder,   "Drivebase"};
    WPI_TalonFX  m_rr_angleMotor{DriveConstants::BRanglePort, "Drivebase"};
    WPI_TalonFX  m_rr_speedMotor{DriveConstants::BRspeedPort, "Drivebase"};
    WPI_CANCoder m_rr_canCoder  {DriveConstants::BRencoder,   "Drivebase"};

    // Swerving PID controllers
    frc2::PIDController m_fl_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};
    frc2::PIDController m_fr_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};
    frc2::PIDController m_rl_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};
    frc2::PIDController m_rr_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};

     frc::ChassisSpeeds speeds;

     frc::SwerveDriveOdometry<4> * odometry; //will need to be initialized later with selected robot start pose
};