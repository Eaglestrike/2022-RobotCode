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
#include "SwerveModule.h"

class Swerve {
    public:
        Swerve(AHRS * nx, DataLogger * logger);
        void Periodic(units::meters_per_second_t joy_x, units::meters_per_second_t joy_y, 
        units::radians_per_second_t joy_theta, units::degree_t navx_yaw);

        frc::ChassisSpeeds getSpeeds();
        frc::Pose2d getPose() { return odometry_->GetPose(); }

        void initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose);
        void updateOdometry(frc::Rotation2d robotAngle, frc::Pose2d robotPose) { odometry_->ResetPosition(robotPose, robotAngle); }

        std::vector<TalonFX *> getTalons(); //for simulation

        wpi::array<frc::SwerveModuleState, 4> getRealModuleStates(); //real as upposed to goal


    private:

    AHRS * m_navx;
    
    DataLogger * m_logger{nullptr};

    void SetPID();

    // Creating my kinematics object using the module locations.
    frc::SwerveDriveKinematics<4> m_kinematics{
        frc::Translation2d{0.3683_m, 0.3683_m}, frc::Translation2d{0.3683_m, -0.3683_m},
        frc::Translation2d{-0.3683_m, 0.3683_m}, frc::Translation2d{-0.3683_m, -0.3683_m}
    };

    SwerveModule flModule_{DriveConstants::FLanglePort, DriveConstants::FLspeedPort, DriveConstants::FLencoder, false, DriveConstants::FLOFF};
    SwerveModule frModule_{DriveConstants::FRanglePort, DriveConstants::FRspeedPort, DriveConstants::FRencoder, true, DriveConstants::FROFF};
    SwerveModule blModule_{DriveConstants::BLanglePort, DriveConstants::BLspeedPort, DriveConstants::BLencoder, true, DriveConstants::BLOFF};
    SwerveModule brModule_{DriveConstants::BRanglePort, DriveConstants::BRspeedPort, DriveConstants::BRencoder, false, DriveConstants::BROFF};

    // Swerving PID controllers
    frc2::PIDController angPID_{DriveConstants::P , DriveConstants::I, DriveConstants::D};
    frc2::PIDController speedPID_{DriveConstants::sP , DriveConstants::sI, DriveConstants::sD};

     frc::ChassisSpeeds speeds_;

     frc::SwerveDriveOdometry<4> * odometry_; //will need to be initialized later with selected robot start pose
};