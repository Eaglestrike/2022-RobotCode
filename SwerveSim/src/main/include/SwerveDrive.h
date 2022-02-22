#include "frc2/command/SubsystemBase.h"
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/trajectory/Trajectory.h>
#include "Constants.h"

class SwerveDrive : public frc2::SubsystemBase {
    public:
        const static double kMaxAngularSpeed = DriveConstants::kMaxChassisRotationSpeed;
        frc::Rotation2d getRotation():

    private:
        bool isFieldOriented;
        double throttle = 0.8; //is this correct?
        double turningThrottle = 0.5;

        frc::SwerveDriveOdometry odometry{DriveConstants::kDriveKinematics, getRotation()};

        double trajectoryTime;
        frc::Trajectory currTraj;



};