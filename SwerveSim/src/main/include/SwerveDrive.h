#include "frc2/command/SubsystemBase.h"
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/Trajectory.h>
#include "AHRS.h"
#include "Constants.h"
#include "SwerveModule.h"

class SwerveDrive : public frc2::SubsystemBase {
    public:
        SwerveDrive();
        const static double kMaxAngularSpeed = DriveConstants::kMaxChassisRotationSpeed;
        //navx utils
        AHRS getNavx() {return mNavX;}
        double getGyroRate() {return mNavX.getRate();}
        frc::Rotation2d getRotation() {return frc::Rotation2d{units::degree_t{getHeading()}};}
        double getTurnRate() { return mNavX.getRate(); }
        double getHeading() { return std::fmod(-mNavX.getAngle(), 360); }
        void resetEncoders();
        void ZeroHeading() {mNavX.Reset();}
       
        //drive/swerve utils
        SwerveModule getSwerveModule(int i) { return mSwerveModules[i]; }
        frc::Pose2d getPose() { return odometry.GetPose(); }

        void drive(double xSpeed, double ySpeed, double rot, bool fieldRelative);
        void setSwerveDriveBrakeMode(boolean on);
        void setModuleStates(frc::SwerveModuleState desiredStates[4]);
        void updateOdometry();
        void resetOdometry(frc::Pose2d pose, frc::Rotation2d rotation);
        //don't really feel like updating sdb rn
        void periodic();
        frc::Pose2d[] getModulePoses();
        //could do sim perioic, sample traj
        
        //traj utils
        void setTrajectoryTime(double trajTime) {trajectoryTime = trajTime;}
        double startTime;
        void setCurrentTrajectory(frc::Trajectory traj);


    private:
        bool isFieldOriented;
        double throttle = 0.8; //is this correct?
        double turningThrottle = 0.5;

        frc::SwerveDriveOdometry odometry{DriveConstants::kDriveKinematics, getRotation()};

        double trajectoryTime;
        frc::Trajectory currTraj;

        //0 fl, 1 fr, 2 bl, 3 br
        SwerveModule mSwerveModules[4] = {
            SwerveModule{0, DriveConstants::FLanglePort, DriveConstants::FLspeedPort, DriveConstants::FLOFF, true, false},
            SwerveModule{1, DriveConstants::FRanglePort, DriveConstants::FRspeedPort, DriveConstants::FROFF, true, false},
            SwerveModule{2, DriveConstants::BLanglePort, DriveConstants::BLspeedPort, DriveConstants::BLOFF, true, false},
            SwerveModule{3, DriveConstants::BRanglePort, DriveConstants::BRspeedPort, DriveConstants::BROFF, true, false}
        };

        AHRS mNavX{frc::SPI::Port::kMXP};
        //can do some sim 


};