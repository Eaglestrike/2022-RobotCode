#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <WheelDrive.h>
#include <SwerveDrive.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <AHRS.h>
#include <frc/Timer.h>
#include <Trajectory.h>
#include <Constants.h>
#include <frc/Compressor.h>
#include "Intake.h"
#include "Shooter.h"
#include "Climber.h"
#include "AutoMode.h"
#include "cameraserver/CameraServer.h"
#include "Swerve.h"
#include "DataLogger.h"


frc::Joystick l_joy{OIConstants::l_joy_port};
frc::Joystick r_joy{OIConstants::r_joy_port};
frc::XboxController xbox{OIConstants::O_joy_port};
frc::PowerDistribution PDH{1, frc::PowerDistribution::ModuleType::kRev};
cs::UsbCamera camera;
 
double m_time = 0;
double m_timeStep = GeneralConstants::timeStep;

//AHRS *navx;

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

  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:

  void joy_val_to_mps(double& val) { val *= 4; }
  void joy_rot_to_rps(double&rot) { rot *= 3*2*PI; }

  AutoMode m_auto;


  Intake m_intake;
  Climber m_climber;
  Swerve swerve;
   Shooter m_shooter{swerve};

   DataLogger * logger{nullptr};

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
  int m_autoSelelected;
};