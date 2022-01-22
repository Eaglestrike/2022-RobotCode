#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  try{
    navx = new AHRS(frc::SPI::Port::kMXP);
  } catch(const std::exception& e){
    std::cout << e.what() <<std::endl;
  }
}


void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}


void Robot::TeleopInit() {
  m_time_step = 0;
  navx->Reset();
}


void Robot::TeleopPeriodic() {
  
  m_time_step += 0.02;
  if(m_time_step <= 2.00){
    m_swerve.Initialize();
  }
  if(m_time_step > 2.00 && m_time_step < 2.10){
    m_swerve.ResetEncoders();
  } 

  else if(m_time_step > 2.10){

    //Deadband 
    double x1, y1, x2;
    x1 = l_joy.GetRawAxis(0);
    y1 = l_joy.GetRawAxis(1) * 0.6;
    x2 = r_joy.GetRawAxis(0);
    x1 = abs(x1) < 0.05 ? 0.0: x1;
    y1 = abs(y1) < 0.05 ? 0.0: y1;
    x2 = abs(x2) < 0.05 ? 0.0: x2;

    m_swerve.Drive(-x1, -y1, -x2, navx->GetYaw(), true);

    m_swerve.UpdateOdometry(navx->GetYaw());
    
    //Button (A)
    if(xbox.GetRawButton(1)){
      std::cout << "reset Odometry" << std::endl;
      m_swerve.ResetOdometry();
    }

    if(xbox.GetRawButton(2)){
      std::cout << "Set Swerve Controller PID" << std::endl;
      // m_swerve.SetDriveControllerROTPID();
      m_swerve.debug();
    }

    frc::SmartDashboard::PutNumber("Y", m_swerve.GetYPosition());
    frc::SmartDashboard::PutNumber("X", m_swerve.GetXPostion());
  }
}


void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}


void Robot::TestInit() {
  m_time_step = 0;
  navx->Reset();
  m_swerve.ResetOdometry();
  m_swerve.GenerateTrajectory_1();
}


void Robot::TestPeriodic() {
  m_time_step += 0.02;

  if(m_time_step <= 1.20){
    m_swerve.Initialize();
  }
  
  if(m_time_step > 1.20 && m_time_step < 1.40){
    m_swerve.ResetEncoders();
  } 

  else if(m_time_step > 1.40){
    m_swerve.UpdateOdometry(navx->GetYaw());
    m_swerve.TrajectoryFollow(navx->GetYaw());

    frc::SmartDashboard::PutNumber("Y", m_swerve.GetYPosition());
    frc::SmartDashboard::PutNumber("X", m_swerve.GetXPostion());    
  }
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif