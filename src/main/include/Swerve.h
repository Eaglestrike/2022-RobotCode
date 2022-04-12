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
#include <Constants.h>
#include "DataLogger.h"
#include "frc/AnalogEncoder.h"
#include "frc/Encoder.h"
#include <frc/MathUtil.h>

class Swerve {
    public:
        Swerve();
        void Init();
        void Periodic(units::meters_per_second_t joy_x, units::meters_per_second_t joy_y, 
        units::radians_per_second_t joy_theta, units::degree_t navx_yaw);

        frc::ChassisSpeeds getSpeeds() { return speeds; }

    // TODO Move this back to private
    DataLogger * logger{nullptr};

    private:

  //  double encoder_to_deg(double count) { return (count/DriveConstants::countsPerRev)*360.0 - 180.0; }

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

    // // Underlying swerve module sensors/actuators
    // WPI_TalonFX  m_fl_angleMotor{DriveConstants::FLanglePort, "Drivebase"};
    // WPI_TalonFX  m_fl_speedMotor{DriveConstants::FLspeedPort, "Drivebase"};
    // WPI_CANCoder m_fl_canCoder  {DriveConstants::FLencoder,   "Drivebase"};
    // WPI_TalonFX  m_fr_angleMotor{DriveConstants::FRanglePort, "Drivebase"};
    // WPI_TalonFX  m_fr_speedMotor{DriveConstants::FRspeedPort, "Drivebase"};
    // WPI_CANCoder m_fr_canCoder  {DriveConstants::FRencoder,   "Drivebase"};
    // WPI_TalonFX  m_rl_angleMotor{DriveConstants::BLanglePort, "Drivebase"};
    // WPI_TalonFX  m_rl_speedMotor{DriveConstants::BLspeedPort, "Drivebase"};
    // WPI_CANCoder m_rl_canCoder  {DriveConstants::BLencoder,   "Drivebase"};
    // WPI_TalonFX  m_rr_angleMotor{DriveConstants::BRanglePort, "Drivebase"};
    // WPI_TalonFX  m_rr_speedMotor{DriveConstants::BRspeedPort, "Drivebase"};
    // WPI_CANCoder m_rr_canCoder  {DriveConstants::BRencoder,   "Drivebase"};

    WPI_TalonSRX  m_fl_angleMotor{DriveConstants::FLanglePort};
    WPI_TalonSRX  m_fl_speedMotor{DriveConstants::FLspeedPort};
    frc::AnalogEncoder m_fl_angle_encoder{1};
    frc::Encoder m_fl_speed_encoder{2, 3};
    WPI_TalonSRX  m_fr_angleMotor{DriveConstants::FRanglePort};
    WPI_TalonSRX  m_fr_speedMotor{DriveConstants::FRspeedPort};
    frc::AnalogEncoder m_fr_angle_encoder{3};
    frc::Encoder m_fr_speed_encoder{4, 5};
    WPI_TalonSRX  m_rl_angleMotor{DriveConstants::BLanglePort};
    WPI_TalonSRX  m_rl_speedMotor{DriveConstants::BLspeedPort};
    frc::AnalogEncoder m_rl_angle_encoder{0};
    frc::Encoder m_rl_speed_encoder{6, 7};
    WPI_TalonSRX  m_rr_angleMotor{DriveConstants::BRanglePort};
    WPI_TalonSRX  m_rr_speedMotor{DriveConstants::BRspeedPort};
    frc::AnalogEncoder m_rr_angle_encoder{2};
    frc::Encoder m_rr_speed_encoder{8, 9};



    // Swerving PID controllers
    frc2::PIDController m_fl_pid{DriveConstants::Pa , DriveConstants::Ia, DriveConstants::Da};
    frc2::PIDController m_fr_pid{DriveConstants::Pa , DriveConstants::Ia, DriveConstants::Da};
    frc2::PIDController m_rl_pid{DriveConstants::Pa , DriveConstants::Ia, DriveConstants::Da};
    frc2::PIDController m_rr_pid{DriveConstants::Pa , DriveConstants::Ia, DriveConstants::Da};

    frc2::PIDController m_fl_speed_pid{DriveConstants::Ps , DriveConstants::Is, DriveConstants::Ds};
    frc2::PIDController m_fr_speed_pid{DriveConstants::Ps , DriveConstants::Is, DriveConstants::Ds};
    frc2::PIDController m_rl_speed_pid{DriveConstants::Ps , DriveConstants::Is, DriveConstants::Ds};
    frc2::PIDController m_rr_speed_pid{DriveConstants::Ps , DriveConstants::Is, DriveConstants::Ds};
    


     frc::ChassisSpeeds speeds;
};