/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose:
  Main Robot File
  Gets driver station input and executes the functions.
*/


#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


// Runs once when robot is enabled
// Sets which color we are (so we know which balls to eject), autonomous mode, and initialize navx
// Add camera server to smartdashboard so we can see the robot's camera feed
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
  m_shooter.GetOdom(m_swerve.copy());
  m_climbing = false;
}


// Runs every 20 ms in every mode
void 
Robot::RobotPeriodic() {
}


// Runs once at the start of autonomous mode
// Generate trajectory for appropriate robot path (depends on auto mode)
// Zero & reset appropriate hardware & motors
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
  m_shooter.Zero();
  m_auto.ResetAuto();
  m_time = 0;
  m_swerve.ResetOdometry();
  m_swerve.Initialize();
  m_intake.Deploy();
  m_shooter.setState(Shooter::State::IDLE);
  m_intake.setState(Intake::State::IDLE);
  
  PDH.ClearStickyFaults();
  navx->Reset();
  m_shooter.enablelimelight();
}


// Called every 20 ms during autonomous period
// Calls appropriate subsystem functions based on state
void 
Robot::AutonomousPeriodic() {
  m_time += m_timeStep;

  m_auto.Periodic(m_time);
  double yaw = navx->GetYaw();

  switch(m_auto.getState()){
    case AutoMode::State::IDLE:
      m_intake.setState(Intake::IDLE);
      m_shooter.setState(Shooter::IDLE);
      m_swerve.Drive(0, 0, 0, 0, true);
      break;
    case AutoMode::State::DRIVEnINTAKE:
      m_intake.setState(Intake::State::RUN);
      m_shooter.setState(Shooter::State::LOAD);
      m_swerve.TrajectoryFollow(yaw, m_auto.getWaypointIndex());
      break;
    case AutoMode::State::SHOOT:
      m_shooter.setState(Shooter::State::SHOOT);
      m_intake.setState(Intake::State::RUN);
      break;
    case AutoMode::State::DRIVE:
      m_swerve.TrajectoryFollow(yaw, m_auto.getWaypointIndex());
      break;
    case AutoMode::State::INTAKE:
      m_intake.setState(Intake::State::RUN);
      m_shooter.setState(Shooter::State::LOAD);
      break;
    default:
      break;
  }
  m_intake.Periodic();
  m_shooter.Periodic(true, yaw);
}


// Runs once at start of teleop
// Sets color bias for shooter, reset shooter & intake states to idle
// Clear sticky faults on hardware
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
  //should only be zeroed once, this is so that we don't have to run auto every time to test teleop
  // m_shooter.Zero();
  
  PDH.ClearStickyFaults();
  m_intake.Deploy();
  m_shooter.Periodic(false, 0);
  m_intake.Periodic();
  m_climber.Initialize();
}


// Called every 20 ms during teleop
void 
Robot::TeleopPeriodic() {
  
  m_time += m_timeStep;

  //Get joystick values, apply deadband (if the abs val is < 0.05 it's probably wind or smth)
  double x1, y1, x2;
  x1 = l_joy.GetRawAxis(0);
  y1 = l_joy.GetRawAxis(1);
  x2 = r_joy.GetRawAxis(0)*0.7;
  x1 = abs(x1) < 0.05 ? 0.0: x1;
  y1 = abs(y1) < 0.05 ? 0.0: y1;
  x2 = abs(x2) < 0.05 ? 0.0: x2;
  double yaw = navx->GetYaw();
  
  //swerve functions, so that robot drive will respond to joystick inputs
  m_swerve.Drive(-x1, -y1, -x2, yaw, true);
  m_swerve.UpdateOdometry(yaw); 

  // frc::SmartDashboard::PutNumber("yaw", navx->GetYaw());
  // frc::SmartDashboard::PutNumber("X",m_swerve.GetXPosition());
  // frc::SmartDashboard::PutNumber("Y",m_swerve.GetYPosition());
  // frc::SmartDashboard::PutNumber("X speed", m_swerve.GetXSpeed());
  // frc::SmartDashboard::PutNumber("Y speed", m_swerve.GetYSpeed());


  //below is the state machine. "state" is which button(s) pressed, robot performs appropriate actins
  //note that since they are else if, the robot is only doing one of these at a time 

  if(xbox.GetBackButtonPressed()){
      m_climbing = !m_climbing;
      m_climber.disableBrake();
      frc::SmartDashboard::PutBoolean("CLIMB", m_climbing);
      m_time_climb = 0;
  }
  //Climbing
  if(m_climbing){ //are we in climbing mode or not
    m_time_climb +=m_timeStep;
    if(m_time_climb < 0.75){
      m_shooter.Climb();
    }
    else if(xbox.GetRawButtonPressed(2)){
      m_climber.ExtendsecondStage();
    }
    else if(xbox.GetRawButtonPressed(3)){
      m_climber.ExtendfirstStage();
    }
    else if(abs(xbox.GetRawAxis(1)) > 0.05){
      out = xbox.GetRawAxis(1);
      m_climber.armExtension(out);
    }
    else if(abs(xbox.GetRawAxis(4)) > 0.2 ){
      m_shooter.Manual(xbox.GetRawAxis(4));
      m_shooter.setState(Shooter::State::MANUAL);
    }
    else {
      m_shooter.Manual(0);
      m_climber.armExtension(0);
    }
  } else {

    //Teleop Operation
    
    // Reset gryoscope yaw value
    //Start button on joystick will reset robot yaw
    if(xbox.GetStartButtonPressed()){
      navx->Reset();
    }

    // Intake 
    else if(r_joy.GetTrigger()){
      m_intake.setState(Intake::State::RUN);
      m_shooter.setState(Shooter::State::LOAD);
    }

    //Shoot
    else if(l_joy.GetTrigger()){
      m_shooter.setState(Shooter::State::SHOOT);
      m_intake.setState(Intake::State::RUN);
    }

    //back button will enable climb
    else if(xbox.GetRawButton(4)){
      m_shooter.setState(Shooter::State::CLIMB);
      m_shooter.zeroHood();
    }
    
    // Button A will outtake
    else if(xbox.GetRawButton(1)){
      m_intake.setState(Intake::State::UNJAM);
      m_shooter.setState(Shooter::BadIdea);
    }

    // toggle intake pnuematics (move intake up/down)
    else if(xbox.GetRawButtonPressed(2)){
      m_intake.toggle();
    }

    // Hard point shooting location at edge of tarmac
    // else if (xbox.GetRawButton(4)){
    //   m_shooter.setState(Shooter::State::Tarmac);
    // }

    else { //make sure shooter and intake aren't intaking if buttons aren't pressed
      // m_shooter.Manual(0);
      m_shooter.setState(Shooter::State::IDLE);
      m_intake.setState(Intake::State::IDLE);
    }
    //call periodic functions so that subsystems will follow actions appropriate to their states
    m_intake.Periodic(); 
    m_shooter.Periodic(false, yaw);
  }
  
}

// Not used but feel free to add stuff here
void 
Robot::TestInit() {}

void 
Robot::TestPeriodic() {}

void 
Robot::DisabledInit() {}

void 
Robot::DisabledPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif