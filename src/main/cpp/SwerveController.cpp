#include <SwerveController.h>

SwerveController::SwerveController(){

}


double 
SwerveController::calculateForward(double goal){
    return m_forwardcontroller.Calculate(m_rotation, goal);
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
SwerveController::setForwardPID(double Kp, double Ki, double Kd){
    m_forwardcontroller.SetP(Kp);
    m_forwardcontroller.SetI(Ki);
    m_forwardcontroller.SetD(Kd);
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

