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

    //gets the states ig
    wpi::array<frc::SwerveModuleState, 4UL> swerveModuleStates = DriveConstants::kDriveKinematics.ToSwerveModuleStates(
        fieldRelative ? 
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::velocity::meters_per_second_t{xSpeed}, units::velocity::meters_per_second_t{ySpeed},
         units::angular_velocity::radians_per_second_t{rot}, getRotation()) 
        : frc::ChassisSpeeds{units::velocity::meters_per_second_t{xSpeed}, units::velocity::meters_per_second_t{ySpeed},
         units::angular_velocity::radians_per_second_t{rot}}); //copied from someone who copied it from 2910's code

    //normalizes the wheel speeds according to max speed
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&swerveModuleStates, units::velocity::meters_per_second_t{DriveConstants::kMaxSpeedMetersPerSec});
    
    //set the desired states 
    mSwerveModules[0].setDesiredState(swerveModuleStates[0]);
    mSwerveModules[1].setDesiredState(swerveModuleStates[1]);
    mSwerveModules[2].setDesiredState(swerveModuleStates[2]);
    mSwerveModules[3].setDesiredState(swerveModuleStates[3]);

}

void SwerveDrive::setSwerveDriveBrakeMode(bool on) {
    for (int i = 0; i < 4; i++) mSwerveModules[i].setBrakeMode(on);
}  


void SwerveDrive::setModuleStates(frc::SwerveModuleState desiredStates[4]) {
    wpi::array<frc::SwerveModuleState, 4UL> ds{*desiredStates}; //must convert it to ds for whatever reason
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&ds, units::velocity::meters_per_second_t{DriveConstants::kMaxSpeedMetersPerSec});
    mSwerveModules[0].setDesiredState(ds[0]);
    mSwerveModules[1].setDesiredState(ds[1]);
    mSwerveModules[2].setDesiredState(ds[2]);
    mSwerveModules[3].setDesiredState(ds[3]);
}

void SwerveDrive::updateOdometry() {
    odometry.Update(getRotation(), mSwerveModules[0].getState(), mSwerveModules[1].getState(), mSwerveModules[2].getState(), mSwerveModules[3].getState());
    for (int i = 0; i < 4; i++) { //idfk what this does but i think it does something helpful
        auto modulePositionFromChassis = DriveConstants::ModulePositions[i].RotateBy(getRotation()) + (getPose().Translation());
        mSwerveModules[i].setPose(frc::Pose2d{modulePositionFromChassis, mSwerveModules[i].getHeading() + (getRotation())});
    }
}

void SwerveDrive::resetOdometry(frc::Pose2d pose, frc::Rotation2d rotation) {
    odometry.ResetPosition(pose, rotation);
    for (int i = 0; i < 4; i++) {
        mSwerveModules[i].setPose(pose);
        mSwerveModules[i].resetEncoders();
    }
}

//i don't wanna update sdb rn

void SwerveDrive::periodic() {
    //could sample traj
    updateOdometry();
    //could update sdb
}

//could do simulation periodic, sample traj

void SwerveDrive::setCurrentTrajectory(frc::Trajectory traj) {
    currTraj = traj;
    startTime = frc::Timer::GetFPGATimestamp().value();
}