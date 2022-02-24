
#include "Robot.h"
#include <iostream>
#include <fmt/core.h>

#include "frc/Timer.h"


#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  frc::SmartDashboard::PutBoolean("Full Extend Pneumatic", m_climber.getFullExtendPneumatic());
  frc::SmartDashboard::PutBoolean("Med Extend Pneumatic", m_climber.getMedExtendPneumatic());
  frc::SmartDashboard::PutNumber("Gearbox master percent out", m_climber.getMasterMotorOutput());
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::SimulationInit() {
  //PhysicsSim::GetInstance().AddTalonFX(climber.getMotor(), 0.75, 3400, false);
}

void Robot::SimulationPeriodic() {
  //PhysicsSim::GetInstance().Run();

}


void Robot::TeleopInit() {
  //climber.SetState(Climber::State::IDLE); //comment in when running climber periodic
  frc::SmartDashboard::PutBoolean("Full Extend Pneumatic", m_climber.getFullExtendPneumatic());
  frc::SmartDashboard::PutBoolean("Med Extend Pneumatic", m_climber.getMedExtendPneumatic());
  frc::SmartDashboard::PutNumber("Gearbox master percent out", m_climber.getMasterMotorOutput());
}


/*** Before running these tests, motor and pneumatic ports must be confirmed***/

void Robot::TeleopPeriodic() {

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  timer.Reset();

}

void Robot::TestPeriodic() {
  climbTestPeriodic();

}

//put this here so we don't have to look at it 
void Robot::climbTestPeriodic() {
  frc::SmartDashboard::PutBoolean("Full Extend Pneumatic", m_climber.getFullExtendPneumatic());
  frc::SmartDashboard::PutBoolean("Med Extend Pneumatic", m_climber.getMedExtendPneumatic());
  frc::SmartDashboard::PutNumber("Gearbox master percent out", m_climber.getMasterMotorOutput());


  m_climber.setTime(timer.Get());
   /** #1: test arm can extend, get extended position
   * Start from fully retracted position (or how the climber will be during most of teleop)
   * Press button until arm is fully extended, record fully extended position as ClimbConstants::motorExtendedPose
   * If printing out that current is too large (>200 amps, can change), & isn't moving, try set inverted = true 
   * Can also increase percent output in function
  **/
  if (xbox.GetRawButton(0)) { //A
    m_climber.extendArmUntilStopped(false);
  }
  /** #2: test arm can retract, retracted position should just be 0
   * start from extended position. This assumes #1 is finished and inverted is correct (do gearboxMaster.SetInverted(true) if true)
   * Just see if it retracts well, can increase percent output in function (esp. if trying to climb)
  **/
  else if (xbox.GetRawButton(1)) { //B
    m_climber.retractArm();
  }
  /** #3: see if pneumatics can go to raised (in between diagonal and vertical) position
   * Climber starts with arms all the way lowered (doesn't really matter, just don't start in midwy position)
   * One pneumatic should retract and the other should extend. Change ports if you don't like the arramgement
  **/
  else if (xbox.GetRawButton(2)) { //X
    m_climber.setArmRaised();
  }
   /** #4: see if pneumatics can go to vertical position
   * Shouldn't be too many hiccups? 
   * See if pneumatics can lift arm. See if ports are correct, but that should have been done by last test
  **/
  else if (xbox.GetRawButton(3)) { //Y
    m_climber.setArmVertical();
  }
   /** #5: See if pneumatics can go to lowered position
   * Start with the arms not in the lowered position. just see if it works
  **/
  else if (xbox.GetRawButton(7)) { //tiny button next to X
    m_climber.setArmLowered();
  }

  //part 2: testing climb walk through

  // /** #6: test if motor can go to set extended pose
  //  * Start from fully retracted position (or how the climber will be during most of teleop)
  //  * Press the button and see if the climber goes to the right position
  //  * If it's too slow or going wrong way or anything, tune PID OR change max ClimbConstants::motorMaxOutput
  // **/
  // else if (xbox.GetRawButton(0)) { //A
  //   m_climber.testRaiseVerticalArm();
  // }
  // /** #7: test if motor can go to set retracted pose
  //  * Start from extended position (just not retracted position)
  //  * Press the button and see if the climber goes to the right position
  //  * If it's too slow or going wrong way or anything, tune PID OR change max ClimbConstants::motorMaxOutput
  // **/
  // else if (xbox.GetRawButton(1)) { //B
  //   m_climber.testRetractVerticalArm();
  // }
  // /** #8: test if arm can extend diagonally
  //  * Not doing hooked for now, just walking through states. this for rest of else-ifs too, for now
  // **/
  // else if (xbox.GetRawButton(2)) { //X
  //   m_climber.testDiagonalExtension();
  // }
  // else if (xbox.GetRawButton(3)) { //Y
  //   m_climber.testDiagonalArmRaise();
  // }
  // else if (xbox.GetRawButton(7)) { //tiny button next to X
  //   m_climber.testBarTraversalFromRaised();
  // }

  else {
    m_climber.Stop();
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
