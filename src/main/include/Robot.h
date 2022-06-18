/* History: 
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Main Robot Header File
  Include the appropriate libraries
  Get the proper objects
*/

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/PowerDistribution.h>
#include <AHRS.h>
#include <frc/Timer.h>
#include <Constants.h>
#include <frc/Compressor.h>
#include "Intake.h"
#include "Shooter.h"
#include "Climber.h"
#include "cameraserver/CameraServer.h"
#include "Swerve.h"
#include "DataLogger.h"


frc::Joystick l_joy{OIConstants::l_joy_port};
frc::Joystick r_joy{OIConstants::r_joy_port};
frc::XboxController xbox{OIConstants::O_joy_port};
frc::PowerDistribution PDH{1, frc::PowerDistribution::ModuleType::kRev};
cs::UsbCamera camera;
 
double m_time = 0;
double m_time_climb = 0;
double m_timeStep = GeneralConstants::timeStep;

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
  void TestInit() override;
  void TestPeriodic() override;


 private:

  //subject to adjustment
  void joy_val_to_mps(double& val) { val *= 4; }
  void joy_rot_to_rps(double& rot) { rot *= 3*2*M_PI; }

  void climbFSM();

  AHRS *m_navx;
  DataLogger *m_logger;
  Limelight *m_limelight;

  //TODO: auto executor
  Swerve m_swerve{m_navx, m_logger};
  Intake m_intake;
  Shooter m_shooter{m_swerve, m_limelight};
  Climber m_climber;
  frc::Pose2d initPose; //this will need to be set as part of auto

  double out;
  bool m_climbing = false;

  frc::SendableChooser<std::string> m_chooser;
  const std::string blueAlliance = "BLUE";
  const std::string redAlliance = "RED";
  //std::string m_autoSelected;

  frc::SendableChooser<int> m_autoMode;
  int mode1 = 1;
  int mode2 = 2;
  int mode3 = 3;
  int mode5 = 5;
  int m_autoSelelected;
};