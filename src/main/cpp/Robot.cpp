#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <frc/geometry/Rotation2d.h>

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

  try{
    navx = new AHRS(frc::SPI::Port::kMXP);
  } catch(const std::exception& e){
    std::cout << e.what() <<std::endl;
  }


 logger = new DataLogger("/home/lvuser/robotlog.log", datalog_fields);
 swerve.logger = logger;
}


void 
Robot::RobotPeriodic() {
}


void 
Robot::AutonomousInit() {

  navx->Reset();

  
}


void 
Robot::AutonomousPeriodic() {

}


void
Robot::TeleopInit() {
 
  m_time = 0;
  navx->Reset();

  swerve.Init();

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
  // dx = -l_joy.GetX();
  // dy = -l_joy.GetY();
  // dtheta = r_joy.GetX();
  dx = abs(dx) < 0.05 ? 0.0: dx;
  dy = abs(dy) < 0.05 ? 0.0: dy;
  dtheta = abs(dtheta) < 0.05 ? 0.0: dtheta;

  joy_val_to_mps(dx);
  joy_val_to_mps(dy);
  joy_rot_to_rps(dtheta);

  swerve.Periodic(
    units::meters_per_second_t{dx},
    units::meters_per_second_t{dy},
    units::radians_per_second_t{dtheta},
    units::degree_t{navx->GetYaw()});
  
  
  // frc::SmartDashboard::PutNumber("Yaw", navx->GetYaw());
  // frc::SmartDashboard::PutNumber("POV", l_joy.GetPOV());

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