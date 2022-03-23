#include "SwerveDrive.h"

#include <algorithm>
#include <iostream>
// Constructor
SwerveDrive::SwerveDrive() {}

// Drive function contains Swerve Inverse Kinematics
void SwerveDrive::Drive(double x1, double y1, double x2, double rot,
                        bool fieldOrient) {
    double r = sqrt(m_L * m_L + m_W * m_W);
    y1 *= -1;
    x1 *= -1;
    x2 *= -1;
    rot *= M_PI / 180;

    if (fieldOrient) {
        m_temp = y1 * cos(rot) + x1 * sin(rot);
        x1 = -y1 * sin(rot) + x1 * cos(rot);
        y1 = m_temp;
    }

    double a = x1 - x2 * (m_L / r);
    double b = x1 + x2 * (m_L / r);
    double c = y1 - x2 * (m_W / r);
    double d = y1 + x2 * (m_W / r);

    double backRightSpeed = sqrt((a * a) + (d * d));
    double backLeftSpeed = sqrt((a * a) + (c * c));
    double frontRightSpeed = sqrt((b * b) + (d * d));
    double frontLeftSpeed = sqrt((b * b) + (c * c));

    // If the commanded wheel speeds are not achievable, maintain their ratio.
    // If we didn't do this, some motors would saturate, breaking the manuevre.
    double max = std::max(std::max(backRightSpeed, backLeftSpeed),
                          std::max(frontRightSpeed, frontLeftSpeed));
    if (max > 1.0) {
        backRightSpeed /= max;
        backLeftSpeed /= max;
        frontRightSpeed /= max;
        frontLeftSpeed /= max;
        // Remove this if it fires often:
        std::cout << "Swerve drive cmd renormalized, prior max was " << max
                  << std::endl;
    }

    double backRightAngle = atan2(a, d) * 180 / M_PI;
    double backLeftAngle = atan2(a, c) * 180 / M_PI;
    double frontRightAngle = atan2(b, d) * 180 / M_PI;
    double frontLeftAngle = atan2(b, c) * 180 / M_PI;

    if (x2 == 0 && x1 == 0 && y1 == 0) {
        m_backRight.Stop();
        m_backLeft.Stop();
        m_frontRight.Stop();
        m_frontLeft.Stop();
        return;
    }

    m_backRight.drive(-backRightSpeed, backRightAngle, DriveConstants::BROFF);
    m_backLeft.drive(backLeftSpeed, backLeftAngle, DriveConstants::BLOFF);
    m_frontRight.drive(-frontRightSpeed, frontRightAngle,
                       DriveConstants::FROFF);
    m_frontLeft.drive(frontLeftSpeed, frontLeftAngle, DriveConstants::FLOFF);
}

// Updates the Odometry
void SwerveDrive::UpdateOdometry(double dt) {
    // NOTE I am explicitly ignoring any polarity flips,
    // because those should be fixed elsewhere, not here.
    // The offsets are sufficient to ensure that modules
    // return robot-relative wheel velocities as (angle, vel)
    // pairs.
    double BR_A = m_backRight.getAngleRad(DriveConstants::BROFF);
    double BR_D = m_backRight.getWheelDistance();
    double BL_A = m_backLeft.getAngleRad(DriveConstants::BLOFF);
    double BL_D = m_backLeft.getWheelDistance();
    double FR_A = m_frontRight.getAngleRad(DriveConstants::FROFF);
    double FR_D = m_frontRight.getWheelDistance();
    double FL_A = m_frontLeft.getAngleRad(DriveConstants::FLOFF);
    double FL_D = m_frontLeft.getWheelDistance();
    m_odometry.updateOdometry(BR_A, BR_D, BL_A, BL_D, FR_A, FR_D, FL_A, FL_D,
                              GetGyroAngleRad(), dt);
}

// Trajectory Following for Swerve
// Cannot turn and move at the same time
// If you want to turn make Waypoint as 0,0,0, theta
void SwerveDrive::TrajectoryFollow(double rot, size_t waypointIndex) {
    size_t index = m_trajectory_1.getIndex();

    bool YError = (abs(GetYPosition() - m_trajectory_1.getY(index)) < 0.3);
    bool XError = (abs(GetXPosition() - m_trajectory_1.getX(index)) < 0.3);
    bool RotError = (abs(rot - m_trajectory_1.getRotation(index)) < 5);

    frc::SmartDashboard::PutNumber(
        "RotError", (abs(rot - m_trajectory_1.getRotation(index))));
    frc::SmartDashboard::PutBoolean("ROTERROR", RotError);

    if (m_trajectory_1.getX(index) == 0 && m_trajectory_1.getY(index) == 0 &&
        m_trajectory_1.getRotation(index) != 0) {
        if (RotError && index < waypointIndex) {
            m_trajectory_1.Progress();
            std::cout << "Gets here once" << std::endl;
            gyro->Reset();
        }
        Drive(0, 0, calcYawStraight(m_trajectory_1.getRotation(index), rot),
              rot, true);
        return;
    }

    m_swerveController.updatePosition(GetYPosition(), GetXPosition(), rot);

    // frc::SmartDashboard::PutNumber("YError", abs(GetYPosition() -
    // m_trajectory_1.getY(index))); frc::SmartDashboard::PutNumber("XError",
    // abs(GetXPosition() - m_trajectory_1.getX(index)));

    double forward =
        m_swerveController.calculateForward(m_trajectory_1.getY(index));
    double strafe =
        m_swerveController.calculateStrafe(m_trajectory_1.getX(index));

    if (YError && XError && RotError && index < waypointIndex) {
        m_trajectory_1.Progress();
    }

    // frc::SmartDashboard::PutNumber("strafe", strafe);

    Drive(strafe, forward,
          calcYawStraight(m_trajectory_1.getRotation(index), rot), rot, true);
}

void SwerveDrive::TrajectoryFollow2(double x_d, double y_d, double theta_d,
                                    double xdot_d, double ydot_d,
                                    double thetadot_d) {
    // We run PIDF controllers on each of x,y,theta independently.
    // We run these controllers to be stateless, which simplified the code.
    // Usually PID controllers are stateful due to integrator and differentiator
    // state. We are not going to use any integral terms (raise your P and tune
    // your paths instead) (though theres a slim chance integral is needed
    // eventually) Differentiator state is handled through noting that D is
    // equivalent to negative sensor rate feedback, which is stateless:
    //
    // output = P * poserror + D * d/dt poserror + F * pathvel
    //        = P * poserror + D * d/dt (pos - pathpos) + F * pathvel
    //        = P * poserror + D * vel - D * pathvel + F * pathvel
    //        = P * poserror + D * vel + (F-D) * pathvel
    //
    // So with a slight adjustment, we can make it work.
    // Another improvement would be to add an acceleration feedforward.

    double xerr = x_d - GetXPosition();
    double yerr = y_d - GetXPosition();
    double terr = theta_d - GetGyroAngleRad();
    // bound angle error properly to (-pi, pi)
    terr = terr - M_2_PI * round(terr / M_2_PI);

    double xvel = GetXSpeed();
    double yvel = GetXSpeed();
    double tvel = GetGyroVelRad();

    using namespace DriveConstants;
    double world_x_cmd = TRAJ_TRANSLATE_P * xerr + TRAJ_TRANSLATE_D * xvel +
                         TRAJ_TRANSLATE_F * xdot_d;
    double world_y_cmd = TRAJ_TRANSLATE_P * yerr + TRAJ_TRANSLATE_D * yvel +
                         TRAJ_TRANSLATE_F * ydot_d;
    double world_t_cmd =
        TRAJ_ROT_P * terr + TRAJ_ROT_D * tvel + TRAJ_ROT_F * thetadot_d;

    // TODO: I am not sure if this argument order is correct.
    Drive(world_y_cmd, world_x_cmd, world_t_cmd, gyro->GetYaw(), true);
}

// Function for generating trajectory_1
void SwerveDrive::GenerateTrajectory_1() {
    m_trajectory_1.clearTrajectory();

    // Default Scoot and Shoot
    // Trajectory::Waypoint p1(1.2, 0, 0, 0);
    // m_trajectory_1.addWaypoint(p1);

    // 2 ball auto
    // Trajectory::Waypoint p1(1.5, 0.1, 0, 0);
    // m_trajectory_1.addWaypoint(p1);

    // 3 ball auto
    Trajectory::Waypoint p1(1.5, 0.15, 0, 0);
    Trajectory::Waypoint p2(0, 0, 0, -110);
    Trajectory::Waypoint p3(5.0, 0.55, 0, 0);
    Trajectory::Waypoint p4(0, 0, 0, 40);
    Trajectory::Waypoint p5(0, 0, 0, 70);
    m_trajectory_1.addWaypoint(p1);
    m_trajectory_1.addWaypoint(p2);
    m_trajectory_1.addWaypoint(p3);
    m_trajectory_1.addWaypoint(p4);
    m_trajectory_1.addWaypoint(p5);

    // 5 ball auto
    // Trajectory::Waypoint p1(1.4, 0.15, 0, 0);
    // Trajectory::Waypoint p2(0, 0, 0, -110);
    // Trajectory::Waypoint p3(5.0, 0.4, 0, 0);
    // Trajectory::Waypoint p4(0, 0, 0, 40);
    // Trajectory::Waypoint p5(10.0, 1.4, 0, 0);
    // Trajectory::Waypoint p6(7.0, 1.4, 0, 0);
    // m_trajectory_1.addWaypoint(p1);
    // m_trajectory_1.addWaypoint(p2);
    // m_trajectory_1.addWaypoint(p3);
    // m_trajectory_1.addWaypoint(p4);
    // m_trajectory_1.addWaypoint(p5);
    // m_trajectory_1.addWaypoint(p6);
}

void SwerveDrive::GenerateTrajectory_2() {
    m_trajectory_2.clearTrajectory();
    // Trajectory::Waypoint p1(0, 0, 0, 90);

    // m_trajectory_2.addWaypoint(p1);
}

// Return y position of the robot
double SwerveDrive::GetYPosition() { return m_odometry.getY(); }

// Return x position of the robot
double SwerveDrive::GetXPosition() { return m_odometry.getX(); }

double SwerveDrive::GetYSpeed() { return m_odometry.getYSpeed(); }

double SwerveDrive::GetXSpeed() { return m_odometry.getXSpeed(); }

// Rotation correction - P controller
double SwerveDrive::calcYawStraight(double targetAngle, double currentAngle) {
    double error = targetAngle - currentAngle;
    return 0.013 * error;
}

// Function to set PID for Swerve Drive Controller
void SwerveDrive::SetDriveControllerPID() { m_swerveController.setPID(); }

void SwerveDrive::SetDriveControllerROTPID() { m_swerveController.setRotPID(); }

// REDUNDANT!! FIX
// Function to Set PID for Swerve Modules
void SwerveDrive::SetPID() {
    m_backRight.setPID();
    m_backLeft.setPID();
    m_frontRight.setPID();
    m_frontLeft.setPID();
}

// Reset Odometry
void SwerveDrive::ResetOdometry() { m_odometry.Reset(); }

// For initialization of Swerve Modules
void SwerveDrive::Initialize() {
    m_backLeft.initialization(m_BL_Offset);
    m_backRight.initialization(m_BR_Offset);
    m_frontRight.initialization(m_FR_Offset);
    m_frontLeft.initialization(m_FL_Offset);
}

// Reset Swerve Module Encoders
void SwerveDrive::ResetEncoders() {
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
}

// Helper Function
void SwerveDrive::InjectNavx(AHRS *navx) { gyro = navx; }

double SwerveDrive::GetGyroAngleRad() {
    assert(gyro != nullptr);
    // flip direction, navx is CW around Z, not CCW
    // but double flip since roborio is upsidedown
    return gyro->GetYaw() * (M_PI / 180.0);
}

double SwerveDrive::GetGyroVelRad() {
    assert(gyro != nullptr);
    // flip direction, navx is CW around Z, not CCW
    // but double flip since roborio is upsidedown
    return gyro->GetRawGyroZ() * (M_PI / 180.0);
}
