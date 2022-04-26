/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Swerve Controller file
  Incoporates fwd, str, and rot control for autonomous swerve movement

  Calculates the swerve drive outputs using PID controllers
  Example: https://www.youtube.com/watch?v=q8wK8l_O_3U&ab_channel=SheehyunKim
*/

#include <SwerveController.h>

//constructor empty for now
SwerveController::SwerveController(){}


//Calculate forward PID output
double 
SwerveController::calculateForward(double goal){
    return std::clamp(m_forwardcontroller.Calculate(m_forward, goal), -0.6, 0.6);
}


//Calculate strafe PID output
double 
SwerveController::calculateStrafe(double goal){
    return std::clamp(-m_strafecontroller.Calculate(m_strafe, goal), -0.6, 0.6);
}


//Calculate rotation PID output
double 
SwerveController::calculateRotation(double goal){
    return std::clamp(m_rotationcontroller.Calculate(m_rotation, goal), -0.5, 0.5);
}


// Set PID for forward and strafe 
// Should Retune if robot changes
void
SwerveController::setPID(){
    double pGain_a = frc::SmartDashboard::GetNumber("P", 0.0);
    frc::SmartDashboard::PutNumber("P", pGain_a);
    double iGain_a = frc::SmartDashboard::GetNumber("I", 0.0);
    frc::SmartDashboard::PutNumber("I", iGain_a);
    double dGain_a = frc::SmartDashboard::GetNumber("D", 0.0);
    frc::SmartDashboard::PutNumber("D", dGain_a);
    m_forwardcontroller.SetPID(pGain_a, iGain_a, dGain_a);
    m_strafecontroller.SetPID(pGain_a, iGain_a, dGain_a);
}


//Set PID for rotation PID
void
SwerveController::setRotPID(){
    double pGain_a = frc::SmartDashboard::GetNumber("Prot", 0.0);
    frc::SmartDashboard::PutNumber("Prot", pGain_a);
    double iGain_a = frc::SmartDashboard::GetNumber("Irot", 0.0);
    frc::SmartDashboard::PutNumber("Irot", iGain_a);
    double dGain_a = frc::SmartDashboard::GetNumber("Drot", 0.0);
    frc::SmartDashboard::PutNumber("Drot", dGain_a);
    m_rotationcontroller.SetPID(pGain_a, iGain_a, dGain_a);
}


//Get position from odometry and update position
void
SwerveController::updatePosition(double forward, double strafe, double rotation){
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;
}

