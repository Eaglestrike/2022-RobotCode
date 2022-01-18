#include <SwerveController.h>

SwerveController::SwerveController(){}


double 
SwerveController::calculateForward(double goal){
    return m_forwardcontroller.Calculate(m_forward, goal);
}


double 
SwerveController::calculateStrafe(double goal){
    return m_strafecontroller.Calculate(m_strafe, goal);
}


double 
SwerveController::calculateRotation(double goal){
    return m_rotationcontroller.Calculate(m_rotation, goal);
}


void
SwerveController::setForwardPID(){
    double pGain_a = frc::SmartDashboard::GetNumber("P angle", 0.0);
    frc::SmartDashboard::PutNumber("P angle", pGain_a);
    double iGain_a = frc::SmartDashboard::GetNumber("I angle", 0.0);
    frc::SmartDashboard::PutNumber("I angle", iGain_a);
    double dGain_a = frc::SmartDashboard::GetNumber("D angle", 0.0);
    frc::SmartDashboard::PutNumber("D angle", dGain_a);
    m_forwardcontroller.SetPID(pGain_a, iGain_a, dGain_a);
}


void 
SwerveController::setStrafePID(double Kp, double Ki, double Kd){

}


void 
SwerveController::setRotationPID(double Kp, double Ki, double Kd){

}


void
SwerveController::updatePosition(double forward, double strafe, double rotation){
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;
}

