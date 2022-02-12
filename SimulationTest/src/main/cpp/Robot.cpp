
#include "Robot.h"
#include <iostream>
#include <fmt/core.h>

#include "frc/Timer.h"


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::SimulationInit() {
  PhysicsSim::GetInstance().AddTalonFX(climber.getMotor(), 0.75, 3400, false);
}

void Robot::SimulationPeriodic() {
  PhysicsSim::GetInstance().Run();

}


void Robot::TeleopInit() {
  // test.setState(testClimbOneBar::State::IDLE);
  waitTest.setState(testWaited::State::IDLE);
}

void Robot::TeleopPeriodic() {
  if (waitTest.getState() == testWaited::State::IDLE) waitTest.setState(testWaited::State::WAITING_FOR_EXTEND_BUTTON);
   if (joystick.GetRawButton(1)) waitTest.setState(testWaited::State::TESTING);
  //  else if (joystick.GetRawButton(2)) test.setState(testClimbOneBar::State::WAITING_FOR_RETRACT_BUTTON);
   else waitTest.setState(testWaited::State::WAITING_FOR_EXTEND_BUTTON);

  waitTest.periodic();
  
  // if (test.getState() == testClimbOneBar::State::IDLE) test.setState(testClimbOneBar::State::WAITING_FOR_EXTEND_BUTTON);
  //  if (joystick.GetRawButton(1)) test.setState(testClimbOneBar::State::TESTING);
  // //  else if (joystick.GetRawButton(2)) test.setState(testClimbOneBar::State::WAITING_FOR_RETRACT_BUTTON);
  //  else test.setState(testClimbOneBar::State::WAITING_FOR_EXTEND_BUTTON);
    
  // test.periodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
