
#include "Robot.h"
#include <iostream>
#include <fmt/core.h>
#include "frc/Timer.h"
#include "PhysicsSim.h"

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  timer.Reset();
  limelight.setLEDMode("OFF");
  frc::SmartDashboard::PutBoolean("Full Extend Pneumatic", m_climber.getFullExtendPneumatic());
  frc::SmartDashboard::PutBoolean("Med Extend Pneumatic", m_climber.getMedExtendPneumatic());
  frc::SmartDashboard::PutNumber("Gearbox master percent out", m_climber.getMasterMotorOutput());
  frc::SmartDashboard::PutNumber("Gearbox master position: ", m_climber.getMotor().GetSelectedSensorPosition());

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
   try{
    navx = new AHRS(frc::SPI::Port::kMXP);
  } catch(const std::exception& e) {
    std::cout << e.what() <<std::endl;
  }

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {}

void Robot::SimulationInit() {
  PhysicsSim::GetInstance().AddTalonFX(m_climber.getMotor(), 0.75, 3400, false);
}

void Robot::SimulationPeriodic() {
  PhysicsSim::GetInstance().Run();

}


void Robot::TeleopInit() {
  //m_climber.SetState(Climber::State::IDLE);
  //std::cout << "here\n";
  m_climber.Stop();
  timer.Reset();
  limelight.setLEDMode("OFF");
  timer.Reset();
  m_climber.SetState(Climber::State::IDLE); //comment in when running climber periodic
  frc::SmartDashboard::PutBoolean("Full Extend Pneumatic", m_climber.getFullExtendPneumatic());
  frc::SmartDashboard::PutBoolean("Med Extend Pneumatic", m_climber.getMedExtendPneumatic());
  frc::SmartDashboard::PutNumber("Gearbox master percent out", m_climber.getMotor().GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("Gearbox master position: ", m_climber.getMotor().GetSelectedSensorPosition());
  m_climber.getMotor().SetNeutralMode(NeutralMode::Coast);
}

void Robot::TeleopPeriodicInit() {
  m_time += m_timeStep;
  if(m_time <= 1.8){
    m_swerve.Initialize();
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


/*** Before running these tests, motor and pneumatic ports must be confirmed***/

double prevPitch = 0;

void Robot::TeleopPeriodic() {

  limelight.setLEDMode("OFF");
  intake.Retract();

  frc::SmartDashboard::PutBoolean("Full Extend Pneumatic", m_climber.getFullExtendPneumatic());
  frc::SmartDashboard::PutBoolean("Med Extend Pneumatic", m_climber.getMedExtendPneumatic());
  frc::SmartDashboard::PutNumber("Gearbox master percent out", m_climber.getMasterMotorOutput());
  frc::SmartDashboard::PutNumber("Gearbox master position: ", m_climber.getMotor().GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Motor current", m_climber.getMotor().GetStatorCurrent());
  frc::SmartDashboard::PutNumber("Pitch", navx->GetPitch());
  frc::SmartDashboard::PutNumber("Delta pitch", navx->GetPitch()-prevPitch);
  frc::SmartDashboard::PutNumber("fpga time", timer.GetFPGATimestamp().value());
  // frc::SmartDashboard::PutNumber("get time", timer.Get().value());

  frc::SmartDashboard::PutNumber("time", timer.GetMatchTime().value());

  if (xbox.GetRawButtonPressed(6) || xbox.GetRawButtonPressed(2)) m_climber.ToggleStopped(); 

  //i++;
  //std::cout << "time: " << i * 0.02 << "\n";
  //m_climber.setTime(units::second_t{i * 0.02});
  // //test.periodic();
   TeleopPeriodicInit();
   m_swerve.Drive(-x1, -y1, -x2, navx->GetYaw(), true);
   m_swerve.UpdateOdometry(navx->GetYaw());

   if (abs(xbox.GetRightY()) > 0.1 && !m_climber.getStopped()) {
     frc::SmartDashboard::PutNumber("joystick out", xbox.GetRightY());
     //std::cout << "going\n";
    if (xbox.GetRightY() > 0) m_climber.getMotor().Set(ControlMode::PercentOutput, 0.25);
    if (xbox.GetRightY() < 0) m_climber.getMotor().Set(ControlMode::PercentOutput, -0.25);
    return;
  }

 // climbTestPeriodic();
  //so A to continue
  //right bumper to stop/start
  //X to retry init climb
  //Y to continue to traversal bar
   m_climber.Periodic(navx->GetPitch()-prevPitch, navx->GetPitch(), timer.GetFPGATimestamp().value(), 
   xbox.GetRawButton(1),  xbox.GetRawButton(1),  xbox.GetRawButton(3),  xbox.GetRawButton(1),  xbox.GetRawButton(4));
  prevPitch = navx->GetPitch();

}

void Robot::DisabledInit() {
  m_climber.getMotor().SetNeutralMode(NeutralMode::Brake);
  m_climber.Stop();
}

void Robot::DisabledPeriodic() {
 m_climber.getMotor().SetNeutralMode(NeutralMode::Brake);
  m_climber.Stop();
}

void Robot::TestInit() {
  timer.Reset();

}


void Robot::TestPeriodic() {
}

//put this here so we don't have to look at it 
void Robot::climbTestPeriodic() {
  m_climber.getBrake().Set(false);

  frc::SmartDashboard::PutBoolean("Full Extend Pneumatic", m_climber.getFullExtendPneumatic());
  frc::SmartDashboard::PutBoolean("Med Extend Pneumatic", m_climber.getMedExtendPneumatic());
  frc::SmartDashboard::PutNumber("Gearbox master percent out", m_climber.getMasterMotorOutput());
  frc::SmartDashboard::PutNumber("Gearbox master position: ", m_climber.getMotor().GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Gearbox master current", m_climber.getMotor().GetStatorCurrent());

  // frc::SmartDashboard::PutNumber("time", timer.GetMatchTime().value());  

  m_climber.setTime(timer.Get());


   /** #1: test arm can extend, get extended position
   * Start from fully retracted position (or how the climber will be during most of teleop)
   * Press button until arm is fully extended, record fully extended position as ClimbConstants::motorExtendedPose
   * If printing out that current is too large (>200 amps, can change), & isn't moving, try set inverted = true 
   * Can also increase percent output in function
  **/
  // if (xbox.GetRawButton(1)) { //A
  //   //std::cout << "here\n";
  //   m_climber.extendArmUntilStopped(true);
  // }
  // /** #2: test arm can retract, retracted position should just be 0
  //  * start from extended position. This assumes #1 is finished and inverted is correct (do gearboxMaster.SetInverted(true) if true)
  //  * Just see if it retracts well, can increase percent output in function (esp. if trying to climb)
  // **/
  // else if (xbox.GetRawButton(2)) { //B
  //   //std::cout << "here\n";
  //   m_climber.retractArm();
  // }
  // /** #3: see if pneumatics can go to raised (in between diagonal and vertical) position
  //  * Climber starts with arms all the way lowered (doesn't really matter, just don't start in midwy position)
  //  * One pneumatic should retract and the other should extend. Change ports if you don't like the arramgement
  // **/
  // else if (xbox.GetRawButton(3)) { //X
  //   m_climber.setArmRaised();
  // }
  //  /** #4: see if pneumatics can go to vertical position
  //  * Shouldn't be too many hiccups? 
  //  * See if pneumatics can lift arm. See if ports are correct, but that should have been done by last test
  // **/
  // else if (xbox.GetRawButton(4)) { //Y
  //   m_climber.setArmVertical();
  // }
  //  /** #5: See if pneumatics can go to lowered position
  //  * Start with the arms not in the lowered position. just see if it works
  // **/
  // else if (xbox.GetRawButton(8)) { //tiny button next to X
  //   m_climber.setArmLowered();
  // }

  // //part 2: testing climb walk through

   /** #6: test if motor can go to set extended pose
   * Start from fully retracted position (or how the climber will be during most of teleop)
   * Press the button and see if the climber goes to the right position
   * If it's too slow or going wrong way or anything, tune PID OR change max ClimbConstants::motorMaxOutput
  **/
  if (xbox.GetRawButton(1)) { //A
    m_climber.testRaiseVerticalArm();
  }
  /** #7: test if motor can go to set retracted pose
   * Start from extended position (just not retracted position)
   * Press the button and see if the climber goes to the right position
   * If it's too slow or going wrong way or anything, tune PID OR change max ClimbConstants::motorMaxOutput
  **/
  else if (xbox.GetRawButton(2)) { //B
    //m_climber.retractArm();
    m_climber.testRetractVerticalArm();
  }
  /** #8: test if arm can extend diagonally
   * Not doing hooked for now, just walking through states. this for rest of else-ifs too, for now
  **/
  else if (xbox.GetRawButton(3)) { //X
    m_climber.testDiagonalExtension();
  }
  else if (xbox.GetRawButton(4)) { //Y
    m_climber.testDiagonalArmRaise();
  }
  else if (xbox.GetRawButton(8)) { //tiny button next to X
    m_climber.testBarTraversalFromRaised();
  }
  else if (xbox.GetRawButton(7)) { //the other one
    m_climber.retractArm();
  }

  else {
    //std::cout << "stopped\n";
    m_climber.Stop();
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
