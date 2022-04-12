#pragma once
#include <string>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <WheelDrive.h>
#include <SwerveDrive.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <Constants.h>
#include "DataLogger.h"
#include <frc/MathUtil.h>
#include <vector>

class Swerve {
    public:
        Swerve();
        void Periodic(units::meters_per_second_t joy_x, units::meters_per_second_t joy_y, 
        units::radians_per_second_t joy_theta, units::degree_t navx_yaw);

        frc::ChassisSpeeds getSpeeds() { return speeds; }

    // TODO Move this back to private
    DataLogger * logger{nullptr};

    std::vector<TalonFX *> getTalons(); //for simulation

    private:

    double ticksToDeg(double ticks) { 
        return frc::InputModulus(ticks, -1024.0, 1024.0) / 1024.0 * 180.0; //SHOULD result in being between -180 and 180
    }

        // Swerve module system
    // SwerveDrive m_swerve;
    // Locations for the swerve drive modules relative to the robot center.
    frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
    frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
    frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
    frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

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
};