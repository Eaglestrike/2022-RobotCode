
#include "Robot.h"
#include <iostream>
#include <fmt/core.h>


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::SimulationInit() {
//  PhysicsSim::GetInstance().AddTalonSRX(shooter.getFlywheel(), 0.75, 3400, false);
//  PhysicsSim::GetInstance().AddTalonSRX(shooter.getTurret(), 0.75, 3400, false);
}

void Robot::SimulationPeriodic() {
  PhysicsSim::GetInstance().Run();

}

void Robot::TeleopInit() {}

frc::Joystick joy{0};

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
