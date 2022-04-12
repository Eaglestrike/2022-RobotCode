#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <frc/geometry/Rotation2d.h>
#include "PhysicsSim.h"

static const DataLogger::DataFields datalog_fields = {
  {"swerve.fl.raw_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.fl.calib_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.fl.target_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.fl.target_speed", DataLogger::DataType::FLOAT64},

  {"swerve.fr.raw_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.fr.calib_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.fr.target_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.fr.target_speed", DataLogger::DataType::FLOAT64},
  
  {"swerve.bl.raw_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.bl.calib_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.bl.target_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.bl.target_speed", DataLogger::DataType::FLOAT64},

  {"swerve.br.raw_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.br.calib_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.br.target_yaw", DataLogger::DataType::FLOAT64},
  {"swerve.br.target_speed", DataLogger::DataType::FLOAT64},

  {"swerve.teleop.dx", DataLogger::DataType::FLOAT64},
  {"swerve.teleop.dy", DataLogger::DataType::FLOAT64},
  {"swerve.teleop.dtheta", DataLogger::DataType::FLOAT64},
  
  {"navx.yaw", DataLogger::DataType::FLOAT64}

};


void 
Robot::RobotInit() {
  m_chooser.SetDefaultOption("Blue", blueAlliance);
  m_chooser.AddOption("RED", redAlliance);
  frc::SmartDashboard::PutData("Alliance Color", &m_chooser);

  m_autoMode.SetDefaultOption("1", mode1);
  m_autoMode.AddOption("2", mode2);
  m_autoMode.AddOption("3", mode3);
  frc::SmartDashboard::PutData("Auto Mode", &m_autoMode);
  
  camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  
  // try{
  //   navx = new AHRS(frc::SPI::Port::kMXP);
  // } catch(const std::exception& e){
  //   std::cout << e.what() <<std::endl;
  // }

  // m_swerve.debug(*navx);
  m_climbing = false;

 //logger = new DataLogger("/home/lvuser/robotlog.log", datalog_fields);
 logger = new DataLogger("~/Downloads/alt", datalog_fields);
 swerve.logger = logger;
}


void 
Robot::RobotPeriodic() {
}

void Robot::SimulationInit() {
  PhysicsSim::GetInstance().AddTalonFXList(swerve.getTalons());
}

void Robot::SimulationPeriodic() {
  PhysicsSim::GetInstance().Run();
}

void 
Robot::AutonomousInit() {
  int mode = m_autoMode.GetSelected();
  m_auto.SetMode(mode);
  if(mode == 1){
    // m_swerve.GenerateTrajectory_1();
  } else if(mode == 2){
    // m_swerve.GenerateTrajectory_2();
  } else if(mode == 3){
    // m_swerve.GenerateTrajectory_3();
  }

  m_auto.ResetAuto();
  m_time = 0;
  // m_swerve.ResetOdometry();
  // m_swerve.Initialize();
  m_intake.Deploy();
  m_shooter.setState(Shooter::State::IDLE);
  m_intake.setState(Intake::State::IDLE);
  m_shooter.Zero();
  PDH.ClearStickyFaults();
  //navx->Reset();
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
      // m_swerve.Drive(0, 0, 0, 0, true);
      break;
    case AutoMode::State::DRIVEnINTAKE:
      m_intake.setState(Intake::State::RUN);
      m_shooter.setState(Shooter::State::LOAD);
      // m_swerve.TrajectoryFollow(navx->GetYaw(), m_auto.getWaypointIndex());
      break;
    case AutoMode::State::SHOOT:
      m_shooter.setState(Shooter::State::SHOOT);
      break;
    case AutoMode::State::DRIVE:
      // m_swerve.TrajectoryFollow(navx->GetYaw(), m_auto.getWaypointIndex());
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
  //navx->Reset();

  m_shooter.setState(Shooter::State::IDLE);
  m_shooter.enablelimelight();
  m_intake.setState(Intake::State::IDLE);
  // m_swerve.Initialize();

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

  auto& dx = logger->get_float64("swerve.teleop.dx");
  auto& dy = logger->get_float64("swerve.teleop.dy");
  auto& dtheta = logger->get_float64("swerve.teleop.dtheta");

  m_time += m_timeStep;

  // Translate linear and angular velocities from joysticks
  // to [dx, dy, dtheta] state-space form.
  // NOTE: The robot coordinate system is +x move forward,
  // and +y points towards the left.
  dx = -l_joy.GetX();
  dy = -l_joy.GetY();
  dtheta = r_joy.GetX();
  dx = abs(dx) < 0.2 ? 0.0: dx;
  dy = abs(dy) < 0.2 ? 0.0: dy;
  dtheta = abs(dtheta) < 0.05 ? 0.0: dtheta;

  joy_val_to_mps(dx);
  joy_val_to_mps(dy);
  joy_rot_to_rps(dtheta);

  swerve.Periodic(
    units::meters_per_second_t{dx},
    units::meters_per_second_t{dy},
    units::radians_per_second_t{dtheta},
    units::degree_t{/*navx->GetYaw()*/0});
  
  // m_swerve.Drive(-x1*0.6, -y1, -x2, navx->GetYaw(), true);
  
  // //Climbing
  // if(m_climbing){
  //   if(xbox.GetRawButtonPressed(2)){
  //     m_climber.ExtendsecondStage();
  //   }
  //   else if(xbox.GetRawButtonPressed(3)){
  //     m_climber.ExtendfirstStage();
      
  //   }
  //   else if(xbox.GetRightBumper()){
  //     out = 0;
  //     m_climber.armExtension(out);
  //   }
  //   else if(abs(xbox.GetRawAxis(1)) > 0.1){
  //     out = xbox.GetRawAxis(1);
  //     if(abs(out) < 0.3){
  //       out = 0;
  //     }
  //     m_climber.armExtension(out);
  //   }
  // } else {
  //   //Teleop Operation
  //   //Intake
  //   if(r_joy.GetTrigger()){
  //     m_intake.setState(Intake::State::RUN);
  //     m_shooter.setState(Shooter::State::LOAD);
  //   }
  //   //Shoot
  //   else if(l_joy.GetTrigger()){
  //     m_shooter.setState(Shooter::State::SHOOT);
  //   }
  //   //Manual turret movement
  //   else if(abs(xbox.GetRawAxis(4)) > 0.2 ){
  //     m_shooter.Manual(xbox.GetRawAxis(4));
  //     m_shooter.setState(Shooter::State::MANUAL);
  //   }
  //   //back button will enable climb
  //   else if(xbox.GetBackButtonPressed()){
  //     m_climbing = true;
  //     m_climber.disableBrake();
  //   }
  //   //button A will reset robot yaw or outtake
  //   else if(xbox.GetStartButtonPressed()){
  //     // navx->Reset();
  //     std::cout << "set pid" << std::endl;
  //     m_shooter.setPID();
  //     // m_shooter.Calibrate();
  //   }
  //   else if(xbox.GetRawButton(1)){
  //     m_intake.setState(Intake::State::UNJAM);
  //     m_shooter.setState(Shooter::BadIdea);
  //   }
  //   else if(l_joy.GetPOV() != -1){
  //     // std::cout << "peek" << std::endl;
  //     m_shooter.peekTurret(navx->GetYaw(), l_joy.GetPOV());
  //   }
  //   else {
  //     m_shooter.setState(Shooter::State::IDLE);
  //     m_intake.setState(Intake::State::IDLE);
  //   }
  //   m_intake.Periodic(); 
  //   m_shooter.Periodic();
    
  // }
  // // frc::SmartDashboard::PutNumber("Yaw", navx->GetYaw());
  // // frc::SmartDashboard::PutNumber("POV", l_joy.GetPOV());

  logger->publish();

}


void 
Robot::TestInit() {
}


void 
Robot::TestPeriodic() {
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