
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
//  PhysicsSim::GetInstance().AddTalonFX(climber.getMotor(), 0.75, 3400, false);
}

void Robot::SimulationPeriodic() {
 // PhysicsSim::GetInstance().Run();

}


void Robot::TeleopInit() {
  //climber.SetState(Climber::State::IDLE); //comment in when running climber periodic
}


/*** Before running these tests, motor and pneumatic ports must be confirmed***/

void Robot::TeleopPeriodic() {
  /** #1: test arm can extend, get extended position
   * Start from fully retracted position (or how the climber will be during most of teleop)
   * Press button until arm is fully extended, record fully extended position as ClimbConstants::motorExtendedPose
   * If printing out that current is too large (>200 amps, can change), & isn't moving, try set inverted = true 
   * Can also increase percent output in function
  **/
  if (/*some button pressed*/false) {
    climber.extendArmUntilStopped(false);
  }
  /** #2: test arm can retract, retracted position should just be 0
   * start from extended position. This assumes #1 is finished and inverted is correct (do gearboxMaster.SetInverted(true) if true)
   * Just see if it retracts well, can increase percent output in function (esp. if trying to climb)
  **/
  else if (/*some other button pressed*/false) {
    climber.retractArm();
  }
  /** #3: see if pneumatics can go to raised (in between diagonal and vertical) position
   * Climber starts with arms all the way lowered (doesn't really matter, just don't start in midwy position)
   * One pneumatic should retract and the other should extend. Change ports if you don't like the arramgement
  **/
  else if (/*some other button pressed*/false) {
    climber.setArmRaised();
  }
   /** #4: see if pneumatics can go to vertical position
   * Shouldn't be too many hiccups? 
   * See if pneumatics can lift arm. See if ports are correct, but that should have been done by last test
  **/
  else if (/*some other button pressed*/false) {
    climber.setArmVertical();
  }
   /** #5: See if pneumatics can go to lowered position
   * Start with the arms not in the lowered position. just see if it works
  **/
  else if (/*some other button pressed*/false) {
    climber.setArmLowered();
  }


  //after this you can do things like setting pids
  //you could also do a thing for setting motor test extended pose, but we might just eyeball that or do percent output


  /** #6: test if motor can go to set extended pose
   * Start from fully retracted position (or how the climber will be during most of teleop)
   * Press the button and see if the climber goes to the right position
   * If it's too slow or going wrong way or anything, tune PID OR change max ClimbConstants::motorMaxOutput
  **/
  else if (/*some other button pressed*/false) {
    climber.testRaiseVerticalArm();
  }
  /** #7: test if motor can go to set retracted pose
   * Start from extended position (just not retracted position)
   * Press the button and see if the climber goes to the right position
   * If it's too slow or going wrong way or anything, tune PID OR change max ClimbConstants::motorMaxOutput
  **/
  else if (/*some other button pressed*/false) {
    climber.testRetractVerticalArm();
  }

  //if you get through all of this and want me to write other things let me know. You can experiment with the state machine
  //the state machine requires a timer in Robot.cpp, as well as navx & choice button values for full functionality
  //i have basic tests in ClimberTestUtils that just sets the states, but you might want to be fancier than that

  //if you do this, think about setting other things like pitch, position, and time tolerances (also hooked current?)

  else {
    climber.Stop();
  }
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
