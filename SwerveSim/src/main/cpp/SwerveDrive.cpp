#include "SwerveDrive.h"

void SwerveDrive::resetEncoders() {
    for (int i = 0; i < 4; i++) {
        mSwerveModules[i].resetEncoders();
    }
}

void SwerveDrive::drive(double xSpeed, double ySpeed, double rot, bool fieldRelative) {
    xSpeed *= DriveConstants::kMaxSpeedMetersPerSec;
    ySpeed *= DriveConstants::kMaxSpeedMetersPerSec;
    rot *= kMaxAngularSpeed;

    auto swerveModuleStates = DriveConstants::kDriveKinematics.ToSwerveModuleStates(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::velocity::meters_per_second_t{xSpeed}, units::velocity::meters_per_second_t{ySpeed}, rot, getRotation()) : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
    
}