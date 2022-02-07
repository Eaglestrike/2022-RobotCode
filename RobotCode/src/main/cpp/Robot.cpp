#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


void 
Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  try{
    navx = new AHRS(frc::SPI::Port::kMXP);
  } catch(const std::exception& e){
    std::cout << e.what() <<std::endl;
  }
}


void 
Robot::RobotPeriodic() {
  //m_compressor.EnableDigital();
}


void 
Robot::AutonomousInit() {}


void 
Robot::AutonomousPeriodic() {}


void
Robot::TeleopInit() {
  m_time = 0;
  navx->Reset();
  m_shooter.setState(Shooter::State::IDLE);
  m_intake.setState(Intake::State::IDLE);
  m_shooter.Periodic();
  m_intake.Periodic();
}




double x1, y1, x2;

void Robot::TeleopPeriodicInit() {
  m_time += m_timeStep;
  if(m_time <= 1.8){
    m_swerve.Initialize();
    m_shooter.Zero();
  }
  else if(m_time > 1.80 && m_time < 1.90){
    m_swerve.ResetEncoders();
  } 

  else if(m_time > 1.90){
    //Deadband 
    x1 = l_joy.GetRawAxis(0) * 0.7;
    y1 = l_joy.GetRawAxis(1);
    x2 = r_joy.GetRawAxis(0);
    x1 = abs(x1) < 0.05 ? 0.0: x1;
    y1 = abs(y1) < 0.05 ? 0.0: y1;
    x2 = abs(x2) < 0.05 ? 0.0: x2;
  }
}

void 
Robot::TeleopPeriodic() {

  TeleopPeriodicInit();  

  m_swerve.Drive(-x1, -y1, -x2, navx->GetYaw(), true);
  m_swerve.UpdateOdometry(navx->GetYaw());
    
  //intaking
  if(r_joy.GetTrigger()){
    m_intake.setState(Intake::State::RUN);
    m_shooter.setState(Shooter::State::LOAD);
    std::cout << "intake" << std::endl;
  }
  //shooting
  else if(l_joy.GetTrigger()){
    m_shooter.setState(Shooter::State::SHOOT);
    std::cout << "shoot" << std::endl;
  }
  //manual turret
  else if(abs(xbox.GetRawAxis(4)) > 0.2 ){
    m_shooter.Manual(xbox.GetRawAxis(4));
    m_shooter.setState(Shooter::State::MANUAL);
    std::cout << "Manual" << std::endl;
  }
  else if(xbox.GetRawButton(1)){
    std::cout << "Set PID for Shooter" << std::endl;
    // m_swerve.ResetOdometry();
    m_shooter.setPID();
    m_shooter.Calibrate();
  } else {
    std::cout << "Idle" << std::endl;
    m_shooter.setState(Shooter::State::IDLE);
    m_intake.setState(Intake::State::IDLE);
  }
  std::cout << "main loop" << std::endl;

  m_intake.Periodic();
  m_shooter.Periodic();

  //frc::SmartDashboard::PutNumber("Y", m_swerve.GetYPosition());
  //frc::SmartDashboard::PutNumber("X", m_swerve.GetXPostion());
}


void 
Robot::TestInit() {
  m_time = 0;
  navx->Reset();
  m_swerve.ResetOdometry();
  m_swerve.GenerateTrajectory_1();
}


void 
Robot::TestPeriodic() {
  m_time += m_timeStep;
  frc::SmartDashboard::PutNumber("Y", m_swerve.GetYPosition());
  frc::SmartDashboard::PutNumber("X", m_swerve.GetXPostion());

  if(m_time <= 1.20){
    m_swerve.Initialize();
  }
  else if(m_time > 1.20 && m_time < 1.40){
    m_swerve.ResetEncoders();
  } 
  else if(m_time > 1.40){
    m_swerve.UpdateOdometry(navx->GetYaw());
    m_swerve.TrajectoryFollow(navx->GetYaw(), false);
  }
}


void 
Robot::DisabledInit() {}


void 
Robot::DisabledPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif