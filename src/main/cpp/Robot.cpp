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

  //select auto mode

  frc::SmartDashboard::PutData("Auto Mode", &m_autoMode);
  
  camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

//  m_limelight = new Limelight();
  
  // try{
  //   m_navx = new AHRS(frc::SPI::Port::kMXP);
  // } catch(const std::exception& e){
  //   std::cout << e.what() <<std::endl;
  // }

  
  // try {
  //   m_logger = new DataLogger(path, datalog_fields);
  // } catch (const std::exception& e){
  //   std::cout << e.what() <<std::endl;
  // }



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

  //call periodic method of appropriate auto

  m_shooter.Zero();
  m_swerve.initializeOdometry(frc::Rotation2d{units::degree_t{m_navx->GetYaw()}}, initPose);
  //initialize auto 
  m_time = 0;
  //do i need swerve initialization?
  m_intake.Deploy();
  m_shooter.setState(Shooter::State::IDLE);
  m_intake.setState(Intake::State::IDLE);
  m_shooter.enablelimelight();
  
  PDH.ClearStickyFaults();
  m_navx->Reset();
  m_shooter.enablelimelight();
}


// Called every 20 ms during autonomous period
// Calls appropriate subsystem functions based on state
void 
Robot::AutonomousPeriodic() {
  m_time += m_timeStep;
  double yaw = m_navx->GetYaw();

  //auto FSM periodic

  m_intake.Periodic();
  m_shooter.Periodic(true);
}


// Runs once at start of teleop
// Sets color bias for shooter, reset shooter & intake states to idle
// Clear sticky faults on hardware
void
Robot::TeleopInit() {
 
  if(m_chooser.GetSelected() == blueAlliance){
    m_shooter.setColor(true);
  } else {
    m_shooter.setColor(false);
  }

  m_time = 0;
  m_navx->Reset();

  m_shooter.setState(Shooter::State::IDLE);
 m_shooter.enablelimelight();
 m_shooter.zeroTurret();
  m_intake.setState(Intake::State::IDLE);

  //REMOVE THIS WHEN AT COMPETITION!
  //should only be zeroed once, this is so that we don't have to run auto every time to test teleop
  //m_shooter.Zero();

  //REMOVE THIS AT COMPETITION!
  m_swerve.initializeOdometry(frc::Rotation2d{units::degree_t{m_navx->GetYaw()}}, initPose);
  
  PDH.ClearStickyFaults();
 // m_intake.Deploy();
  //m_shooter.Periodic(false);
  m_intake.Periodic();
  m_climber.Initialize();

}


// Called every 20 ms during teleop
void 
Robot::TeleopPeriodic() {

  m_logger->publish(); //should probably go at the end, just putting here to avoid premature returns
   
  m_time += m_timeStep;

  //Get joystick values, apply deadband (if the abs val is < 0.05 it's probably wind or smth)
  
  double dx = -l_joy.GetX();
  double dy = -l_joy.GetY();
  double dtheta = r_joy.GetX();
  dx = abs(dx) < 0.05 ? 0.0: dx;
  dy = abs(dy) < 0.05 ? 0.0: dy;
  dtheta = abs(dtheta) < 0.05 ? 0.0: dtheta;

  joy_val_to_mps(dx);
  joy_val_to_mps(dy);
  joy_rot_to_rps(dtheta);

  m_swerve.Periodic(
    units::meters_per_second_t{dy},
    units::meters_per_second_t{dx},
    units::radians_per_second_t{dtheta},
    units::degree_t{m_navx->GetYaw()});

    frc::Pose2d pose = m_limelight->getPose(m_navx->GetYaw(), m_shooter.getTurretAngle());
    frc::SmartDashboard::PutNumber("Pose x", pose.X().value());
    frc::SmartDashboard::PutNumber("Pose y", pose.Y().value());

    // //A
    // if (xbox.GetRawButton(1)) {
    //   m_swerve.test1ms();
    // }
    // else if (xbox.GetRawButton(2)) {
    //   m_swerve.test2_5ms();
    // }
    // else if (xbox.GetRawButton(3)) {
    //   m_swerve.test4ms();
    // }
    // else {
    //    m_swerve.Periodic(
    //   units::meters_per_second_t{0},
    //   units::meters_per_second_t{0},
    //   units::radians_per_second_t{0},
    //   units::degree_t{m_navx->GetYaw()});
    // }


    // frc::ChassisSpeeds speeds = m_swerve.getSpeeds();
    // frc::SmartDashboard::PutNumber("x speed", speeds.vx.value());
    // frc::SmartDashboard::PutNumber("y speed", speeds.vy.value());

    // m_logger->get_float64("x_vel") = speeds.vx.value();
    // m_logger->get_float64("y_vel") = speeds.vy.value();

  return; //for swerve testing, don't want to do other stuff


  //below is the state machine. "state" is which button(s) pressed, robot performs appropriate actins
  //note that since they are else if, the robot is only doing one of these at a time 

  if(xbox.GetBackButtonPressed()){  //enter climbing sate
      m_climbing = !m_climbing;
      m_climber.disableBrake();
      m_time_climb = 0;
  }
  //Climbing
  if(m_climbing){ //are we in climbing mode or not
    climbFSM();
  } else {
    //Teleop Operation
    
    // Reset gryoscope yaw value
    //Start button on joystick will reset robot yaw
    if(xbox.GetStartButtonPressed()){
      m_navx->Reset();
    }

    // Intake 
    else if(r_joy.GetTrigger()){
     // m_intake.setState(Intake::State::RUN); //will this cause a problem if the channel should not run?
      m_shooter.setState(Shooter::State::LOAD);
    }

    //Shoot
    else if(l_joy.GetTrigger()){
      m_shooter.setState(Shooter::State::SHOOT);
      m_intake.setState(Intake::State::RUN);
    }

    //back button will enable climb
    //is raw button 4 the back button?
    else if(xbox.GetRawButton(4)){
      m_shooter.setState(Shooter::State::CLIMB);
      m_shooter.zeroHood();
    }
    
    // Button A will outtake
    else if(xbox.GetRawButton(1)){
      m_intake.setState(Intake::State::UNJAM);
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
    m_shooter.Periodic(false);
  }
  
}

void Robot::climbFSM() {
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
}

// Not used but feel free to add stuff here
void 
Robot::TestInit() {}

void 
Robot::TestPeriodic() {}

void 
Robot::DisabledInit() {}

void 
Robot::DisabledPeriodic() {
 // m_swerve.DisabledPeriodic(nullptr);
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif