#include "SwerveDrive.h"
#include <iostream>
//Constructor
SwerveDrive::SwerveDrive(){}


//Drive function contains Swerve Inverse Kinematics
void 
SwerveDrive::Drive(double x1, double y1, double x2, double rot, bool fieldOrient){
    double r = sqrt(m_L*m_L + m_W*m_W);
    y1 *= -1;
    x1 *= -1;
    x2 *= -1;
    rot *= M_PI/180;

    if(fieldOrient){
        m_temp = y1*cos(rot) + x1*sin(rot);
        x1 = -y1*sin(rot) + x1*cos(rot);
        y1 = m_temp;
    }

    double a = x1 - x2 * (m_L/r);
    double b = x1 + x2 * (m_L/r);
    double c = y1 - x2 * (m_W/r);
    double d = y1 + x2 * (m_W/r);

    double backRightSpeed = sqrt((a*a) + (d*d));
    double backLeftSpeed = sqrt((a*a) + (c*c));
    double frontRightSpeed = sqrt((b*b) + (d*d));
    double frontLeftSpeed = sqrt((b*b) + (c*c));

    double backRightAngle = atan2(a, d) * 180/ M_PI;
    double backLeftAngle = atan2(a, c) *180/ M_PI;
    double frontRightAngle = atan2(b, d) *180/ M_PI;
    double frontLeftAngle = atan2(b, c) *180/ M_PI;

    if(x2 == 0 && x1 == 0 && y1 == 0){
        m_backRight.Stop();
        m_backLeft.Stop();
        m_frontRight.Stop();
        m_frontLeft.Stop();
        return;
    }

    m_backRight.drive(-backRightSpeed, backRightAngle,
        DriveConstants::BROFF);
    m_backLeft.drive(backLeftSpeed, backLeftAngle,
        DriveConstants::BLOFF);
    m_frontRight.drive(-frontRightSpeed, frontRightAngle,
        DriveConstants::FROFF);
    m_frontLeft.drive(frontLeftSpeed, frontLeftAngle, 
        DriveConstants::FLOFF);
}


//Updates the Odometry
void
SwerveDrive::UpdateOdometry(double theta){
    double BR_A = m_backRight.getAngle(DriveConstants::BROFF);
    double BR_V = m_backRight.getVelocity();
    double BL_A = m_backLeft.getAngle(DriveConstants::BLOFF);
    double BL_V = m_backLeft.getVelocity();
    double FR_A = m_frontRight.getAngle(DriveConstants::FROFF);
    double FR_V = m_frontRight.getVelocity();
    double FL_A = m_frontLeft.getAngle(DriveConstants::FLOFF);
    double FL_V = m_frontLeft.getVelocity();
    frc::SmartDashboard::PutNumber("BRA", BR_A);
    frc::SmartDashboard::PutNumber("BRV", -BR_V);
    frc::SmartDashboard::PutNumber("BLA", BL_A);
    frc::SmartDashboard::PutNumber("BLV", BL_V);
    frc::SmartDashboard::PutNumber("FRA", -FR_A);
    frc::SmartDashboard::PutNumber("FRV", -FR_V);
    frc::SmartDashboard::PutNumber("FLA", -FL_A);
    frc::SmartDashboard::PutNumber("FLV", FL_V);
    std::cout << FR_V << std::endl;
    m_odometry.updateOdometry(BR_A, BR_V, BL_A, BL_V, FR_A, FR_V,
        FL_A, FL_V, theta);
}


//Trajectory Following for Swerve
//Cannot turn and move at the same time
// ok I fixed odometry so it should rotate and move now??
//If you want to turn make Waypoint as 0,0,0, theta
void
SwerveDrive::TrajectoryFollow(double rot, size_t waypointIndex){
    size_t index = m_trajectory_1.getIndex();
    
    bool YError = (abs(GetYPosition() - m_trajectory_1.getY(index)) < 0.3);
    bool XError = (abs(GetXPosition() - m_trajectory_1.getX(index)) < 0.3);
    bool RotError = (abs(rot - m_trajectory_1.getRotation(index)) < 5);

    // frc::SmartDashboard::PutNumber("RotError", (abs(rot - m_trajectory_1.getRotation(index))));
    // frc::SmartDashboard::PutBoolean("ROTERROR", RotError);

//     if(m_trajectory_1.getX(index) == 0 &&
//         m_trajectory_1.getY(index) == 0 &&
//         m_trajectory_1.getRotation(index) != 0){
//         if(RotError && index < waypointIndex){
//             m_trajectory_1.Progress();
//             // std::cout << "Gets here once" << std::endl;
//             // gyro->Reset();
//         }
        Drive(0, 0, calcYawStraight(m_trajectory_1.getRotation(index), rot), rot, true);
        return;
    }

    // m_swerveController.updatePosition(GetYPosition(), GetXPosition(), rot);

    double forward = m_swerveController.calculateForward(m_trajectory_1.getY(index));
    double strafe = m_swerveController.calculateStrafe(m_trajectory_1.getX(index));

    if( YError && XError && RotError && index < waypointIndex){
        m_trajectory_1.Progress();
    }

    //frc::SmartDashboard::PutNumber("strafe", strafe);

    UpdateOdometry(rot);
    Drive(strafe, forward, calcYawStraight(m_trajectory_1.getRotation(index), rot), rot, true);
}


//Function for generating trajectory_1
void
SwerveDrive::GenerateTrajectory_1(){
    m_trajectory_1.clearTrajectory();
    // Default Scoot and Shoot
    Trajectory::Waypoint p1(1.2, 0, 0, 0);
    m_trajectory_1.addWaypoint(p1);
}


void
SwerveDrive::GenerateTrajectory_2(){
     m_trajectory_1.clearTrajectory();
    // 2 ball auto
    Trajectory::Waypoint p1(1.5, 0.05, 0, 0);
    m_trajectory_1.addWaypoint(p1);
}


void
SwerveDrive::GenerateTrajectory_3(){
    m_trajectory_1.clearTrajectory();
    // 3 ball auto
    Trajectory::Waypoint p1(1.25, 0.15, 0, 0);
    Trajectory::Waypoint p2(0, 0, 0, -110);
    Trajectory::Waypoint p3(4.2, 0.6, 0, 0); // 0.3 doesn't go enough in library testing to the right
    Trajectory::Waypoint p4(0, 0, 0, 40);
    Trajectory::Waypoint p5(0, 0, 0, 160);
    m_trajectory_1.addWaypoint(p1);
    m_trajectory_1.addWaypoint(p2);
    m_trajectory_1.addWaypoint(p3);  
    m_trajectory_1.addWaypoint(p4);
    m_trajectory_1.addWaypoint(p5);
}


void
SwerveDrive::GenerateTrajectory_5(){
    m_trajectory_1.clearTrajectory();
    // 5 ball auto
    Trajectory::Waypoint p1(1.25, 0.15, 0, 0);
    Trajectory::Waypoint p2(0, 0, 0, -110);
    Trajectory::Waypoint p3(4.2, 0.9, 0, 0);
    Trajectory::Waypoint p4(0, 0, 0, 40);
    Trajectory::Waypoint p5(7.3, 2.0, 0, 0);
    Trajectory::Waypoint p6(4.0, 1.3, 0, 0);
    m_trajectory_1.addWaypoint(p1);
    m_trajectory_1.addWaypoint(p2);
    m_trajectory_1.addWaypoint(p3);  
    m_trajectory_1.addWaypoint(p4);
    m_trajectory_1.addWaypoint(p5);
    m_trajectory_1.addWaypoint(p6);
}

//Return y position of the robot
double
SwerveDrive::GetYPosition(){
    return m_odometry.getY();
}


//Return x position of the robot
double
SwerveDrive::GetXPosition(){
    return m_odometry.getX();
}


double
SwerveDrive::GetYSpeed(){
    return m_odometry.getYSpeed();
}


double
SwerveDrive::GetXSpeed(){
    return m_odometry.getXSpeed();
}


//Rotation correction - P controller
double 
SwerveDrive::calcYawStraight(double targetAngle, double currentAngle){
    double error = targetAngle - currentAngle;
    return 0.013*error;
}


//Function to set PID for Swerve Drive Controller
void
SwerveDrive::SetDriveControllerPID(){
    m_swerveController.setPID();
}


void
SwerveDrive::SetDriveControllerROTPID(){
    m_swerveController.setRotPID();
}


//REDUNDANT!! FIX
//Function to Set PID for Swerve Modules
void 
SwerveDrive::SetPID(){
    m_backRight.setPID();
    m_backLeft.setPID();
    m_frontRight.setPID();
    m_frontLeft.setPID();
}


//Reset Odometry
void
SwerveDrive::ResetOdometry(){
    m_odometry.Reset();
}


//For initialization of Swerve Modules
void
SwerveDrive::Initialize(){
    m_backLeft.initialization(m_BL_Offset);
    m_backRight.initialization(m_BR_Offset);
    m_frontRight.initialization(m_FR_Offset);
    m_frontLeft.initialization(m_FL_Offset);    
}


//Reset Swerve Module Encoders
void
SwerveDrive::ResetEncoders(){
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
}


//Helper Function
void 
SwerveDrive::debug(AHRS &navx){
    gyro = navx;
}


Odometry 
SwerveDrive::copy(){
    return m_odometry;
}
