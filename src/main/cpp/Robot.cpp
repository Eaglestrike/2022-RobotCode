#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


void 
Robot::RobotInit() {
  m_chooser.SetDefaultOption("Blue", blueAlliance);
  m_chooser.AddOption("RED", redAlliance);
  frc::SmartDashboard::PutData("Alliance Color", &m_chooser);

  m_autoMode.SetDefaultOption("1", mode1);
  m_autoMode.AddOption("2", mode2);
  m_autoMode.AddOption("3", mode3);
  m_autoMode.AddOption("5", mode5);
  frc::SmartDashboard::PutData("Auto Mode", &m_autoMode);
  
  camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  
  try{
    navx = new AHRS(frc::SPI::Port::kMXP);
  } catch(const std::exception& e){
    std::cout << e.what() <<std::endl;
  }

  m_swerve.debug(*navx);
  m_climbing = false;
}


void 
Robot::RobotPeriodic() {
}


void 
Robot::AutonomousInit() {
  int mode = m_autoMode.GetSelected();
  m_auto.SetMode(mode);
  if(mode == 1){
    m_swerve.GenerateTrajectory_1();
  } else if(mode == 2){
    m_swerve.GenerateTrajectory_2();
  } else if(mode == 3){
    m_swerve.GenerateTrajectory_3();
  } else if(mode == 5){
    m_swerve.GenerateTrajectory_5();
  }

  m_auto.ResetAuto();
  m_time = 0;
  m_swerve.ResetOdometry();
  m_swerve.Initialize();
  m_intake.Deploy();
  m_shooter.setState(Shooter::State::IDLE);
  m_intake.setState(Intake::State::IDLE);
  m_shooter.Zero();
  PDH.ClearStickyFaults();
  navx->Reset();
  m_shooter.enablelimelight();
  //This might not work??
  
}


void 
Robot::AutonomousPeriodic() {
  m_time += m_timeStep;

  m_auto.Periodic(m_time);

  switch(m_auto.getState()){
    case AutoMode::State::IDLE:
      m_intake.setState(Intake::IDLE);
      m_shooter.setState(Shooter::IDLE);
      m_swerve.Drive(0, 0, 0, 0, true);
      break;
    case AutoMode::State::DRIVEnINTAKE:
      m_intake.setState(Intake::State::RUN);
      m_shooter.setState(Shooter::State::LOAD);
      m_swerve.TrajectoryFollow(navx->GetYaw(), m_auto.getWaypointIndex());
      break;
    case AutoMode::State::SHOOT:
      m_shooter.setState(Shooter::State::SHOOT);
      break;
    case AutoMode::State::DRIVE:
      m_swerve.TrajectoryFollow(navx->GetYaw(), m_auto.getWaypointIndex());
      break;
    case AutoMode::State::INTAKE:
      m_intake.setState(Intake::State::RUN);
      m_shooter.setState(Shooter::State::LOAD);
      break;
    default:
      break;
  }
  m_intake.Periodic();
  m_shooter.Periodic();
}


void
Robot::TeleopInit() {
  m_climber.Initialize();

  if(m_chooser.GetSelected() == blueAlliance){
    m_shooter.setColor(true);
  } else {
    m_shooter.setColor(false);
  }

  m_time = 0;
  navx->Reset();

  m_shooter.setState(Shooter::State::IDLE);
  m_shooter.enablelimelight();
  m_intake.setState(Intake::State::IDLE);
  m_swerve.Initialize();

  //REMOVE THIS WHEN AT COMPETITION!
  m_shooter.Zero();
  
  PDH.ClearStickyFaults();
  m_intake.Deploy();
  m_shooter.Periodic();
  m_intake.Periodic();
  m_climber.Initialize();
}


void 
Robot::TeleopPeriodic() {
  
  m_time += m_timeStep;
  double x1, y1, x2;
  x1 = l_joy.GetRawAxis(0);
  y1 = l_joy.GetRawAxis(1);
  x2 = r_joy.GetRawAxis(0)*0.7;
  x1 = abs(x1) < 0.05 ? 0.0: x1;
  y1 = abs(y1) < 0.05 ? 0.0: y1;
  x2 = abs(x2) < 0.05 ? 0.0: x2;
  
  // frc::SmartDashboard::PutNumber("X",m_swerve.GetXPosition());
  // frc::SmartDashboard::PutNumber("Y",m_swerve.GetYPosition());
  m_swerve.Drive(-x1, -y1, -x2, navx->GetYaw(), true);
  
  //Climbing
  if(m_climbing){
    if(xbox.GetRawButtonPressed(2)){
      m_climber.ExtendsecondStage();
    }
    else if(xbox.GetRawButtonPressed(3)){
      m_climber.ExtendfirstStage();
      
    }
    else if(xbox.GetRightBumper()){
      out = 0;
      m_climber.armExtension(out);
    }
    else if(abs(xbox.GetRawAxis(1)) > 0.1){
      out = xbox.GetRawAxis(1);
      if(abs(out) < 0.3){
        out = 0;
      }
      m_climber.armExtension(out);
    }
  } else {
    //Teleop Operation
    //Intake
    if(r_joy.GetTrigger()){
      m_intake.setState(Intake::State::RUN);
      m_shooter.setState(Shooter::State::LOAD);
    }
    //Shoot
    else if(l_joy.GetTrigger()){
      m_shooter.setState(Shooter::State::SHOOT);
    }
    //Peeking the turret feild oriented
    else if(l_joy.GetPOV() != -1){
      m_shooter.peekTurret(navx->GetYaw(), l_joy.GetPOV());
      m_shooter.setState(Shooter::State::PEEK);
    }
    //back button will enable climb
    else if(xbox.GetBackButtonPressed()){
      m_climbing = true;
      m_climber.disableBrake();
    }
    //Start button on joystick will reset robot yaw
    else if(xbox.GetStartButtonPressed()){
      navx->Reset();
      // m_shooter.setPID();
      // m_shooter.Calibrate();
    }
    // Button A will outtake
    else if(xbox.GetRawButton(1)){
      m_intake.setState(Intake::State::UNJAM);
      m_shooter.setState(Shooter::BadIdea);
    }
    else {
      m_shooter.setState(Shooter::State::IDLE);
      m_intake.setState(Intake::State::IDLE);
    }
    m_intake.Periodic(); 
    m_shooter.Periodic();
  }
  

}


void 
Robot::TestInit() {
}


void 
Robot::TestPeriodic() {
  //Manual turret movement
    //Remove this once peeking turret works
    // else if(abs(xbox.GetRawAxis(4)) > 0.2 ){
    //   m_shooter.Manual(xbox.GetRawAxis(4));
    //   m_shooter.setState(Shooter::State::MANUAL);
    // }
}


void 
Robot::DisabledInit() {
  // m_climber.enableBrake();
  // m_climber.whenDisabled();
}


void 
Robot::DisabledPeriodic() {}



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif