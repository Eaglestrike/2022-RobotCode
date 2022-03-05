#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <WheelDrive.h>
#include <SwerveDrive.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <AHRS.h>
#include <frc/Timer.h>
#include <Trajectory.h>
#include <Constants.h>
#include <frc/Compressor.h>
#include "Intake.h"
#include "Climber.h"
#include "Shooter.h"
#include "Autonomous.h"
<<<<<<< HEAD
#include "testPassStateEnoughTime.h"
=======
#include "testClimbOneBar.h"
#include "Limelight.h"
#include <frc/Timer.h>
>>>>>>> 153137d7462779dbe95f5a48ef39b3f82962bbdb


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void swerveTestPeriodic();
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  void TeleopPeriodicInit();
  void climbTestPeriodic();


 private:

  //SwerveDrive m_swerve;
  // Intake m_intake;
  // Shooter m_shooter;
  Climber m_climber;

  double x1, y1, x2;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
  Limelight limelight;
  //testClimbOneBar test{m_climber};

  AHRS *navx;
  SwerveDrive m_swerve;
  Intake intake;

  frc::Joystick l_joy{OIConstants::l_joy_port};
  frc::Joystick r_joy{OIConstants::r_joy_port};
  frc::XboxController xbox{OIConstants::O_joy_port};
  //frc::Compressor m_compressor{frc::PneumaticsModuleType::REVPH};

  frc::Timer timer;

  double m_time = 0;
  double m_timeStep = GeneralConstants::timeStep;


  testPassStateEnoughTime tpset{m_climber};

};