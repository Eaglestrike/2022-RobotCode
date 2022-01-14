#include "SwerveDrive.h"

SwerveDrive::SwerveDrive(){}


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

    frc::SmartDashboard::PutNumber("Rotation",rot);

    m_backRight.drive(backRightSpeed, backRightAngle);
    m_backLeft.drive(-backLeftSpeed, backLeftAngle);
    m_frontRight.drive(frontRightSpeed, frontRightAngle);
    m_frontLeft.drive(-frontLeftSpeed, frontLeftAngle);
}


void 
SwerveDrive::ResetEncoders(){
    m_backRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_frontLeft.resetEncoder();
}


void
SwerveDrive::UpdateOdometry(double ROT, double theta){
    double BR_A = m_backRight.getAngle();
    double BR_V = m_backRight.getVelocity();
    double BL_A = m_backLeft.getAngle();
    double BL_V = m_backLeft.getVelocity();
    double FR_A = m_frontRight.getAngle();
    double FR_V = m_frontRight.getVelocity();
    double FL_A = m_frontLeft.getAngle();
    double FL_V = m_frontLeft.getVelocity();
    m_odometry.updateOdometry(BR_A, BR_V, BL_A, BL_V, FR_A, FR_V,
        FL_A, FL_V, ROT, theta);

    frc::SmartDashboard::PutNumber("X", m_odometry.getX());
    frc::SmartDashboard::PutNumber("Y", m_odometry.getY());
}

void
SwerveDrive::TrajectoryFollow(double fwd, double str, double rot){
    m_swerveController.updatePosition(fwd, str, rot);
    double forward = m_swerveController.calculateForward(10);
    Drive(forward, 0, 0, 0, true);
}


void
SwerveDrive::SetDriveControllerPID(){
    double pGain_a = frc::SmartDashboard::GetNumber("P", 0.0);
    frc::SmartDashboard::PutNumber("P", pGain_a);
    double iGain_a = frc::SmartDashboard::GetNumber("I", 0.0);
    frc::SmartDashboard::PutNumber("I", iGain_a);
    double dGain_a = frc::SmartDashboard::GetNumber("D", 0.0);
    frc::SmartDashboard::PutNumber("D", dGain_a);
    m_swerveController.setForwardPID(pGain_a, iGain_a, dGain_a);
}


void 
SwerveDrive::SetPID(){
    m_backRight.setPID();
    m_backLeft.setPID();
    m_frontRight.setPID();
    m_frontLeft.setPID();
}


void 
SwerveDrive::TestPWM(){
    m_backLeft.initialization();
}


void
SwerveDrive::ResetOdometry(){
    m_odometry.Reset();
}