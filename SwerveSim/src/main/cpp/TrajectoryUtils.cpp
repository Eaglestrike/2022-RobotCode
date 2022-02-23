#include "TrajectoryUtils.h"
#include <vector>

frc2::SwerveControllerCommand<4> TrajectoryUtils::generateSwerveCommand(SwerveDrive swerveDrive, frc::Trajectory traj) {
    frc2::SwerveControllerCommand<4> swerveCommand{
        traj,
        [&]() {return swerveDrive.getPose(); },
        DriveConstants::kDriveKinematics,
        frc::PIDController{AutoConstants::kpXController, 0, 0},
        frc::PIDController{AutoConstants::kpYController, 0, 0},
        frc::ProfiledPIDController<units::radians>{AutoConstants::kpThetaController, 0, 0, 
            AutoConstants::kThetaControllerConstraints},
        [&](auto moduleStates) {return SwerveDrive::setModuleStates; },
        {&swerveDrive}  
    };
    return swerveCommand;
}

frc2::SwerveControllerCommand<4> generateSwerveCommand(SwerveDrive swerveDrive, frc::Trajectory traj, std::function<frc::Rotation2d()> robotHeading) {
    frc2::SwerveControllerCommand<4> swerveCommand{
        traj,
        [&]() {return swerveDrive.getPose(); },
        DriveConstants::kDriveKinematics,
        frc::PIDController{AutoConstants::kpXController, 0, 0},
        frc::PIDController{AutoConstants::kpYController, 0, 0},
        frc::ProfiledPIDController<units::radians>{AutoConstants::kpThetaController, 0, 0, 
            AutoConstants::kThetaControllerConstraints},
        robotHeading,
        [&](auto moduleStates) {return SwerveDrive::setModuleStates; },
        {&swerveDrive}  
    };
    return swerveCommand;
}