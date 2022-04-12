#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <AHRS.h>
#include <frc/Timer.h>
#include <Constants.h>
#include <frc/Compressor.h>
#include "cameraserver/CameraServer.h"
#include "Swerve.h"
#include "DataLogger.h"


// frc::Joystick l_joy{OIConstants::l_joy_port};
// frc::Joystick r_joy{OIConstants::r_joy_port};
// frc::XboxController xbox{OIConstants::O_joy_port};
// frc::PowerDistribution PDH{1, frc::PowerDistribution::ModuleType::kRev};
//cs::UsbCamera camera;
 
double m_time = 0;
double m_timeStep = GeneralConstants::timeStep;

AHRS *navx;

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

  double PI = 3.141593;

  void joy_val_to_mps(double& val) { val *= 4; }
  void joy_rot_to_rps(double&rot) { rot *= 3*2*PI; }

  Swerve swerve;

   DataLogger * logger{nullptr};

  double out;
  bool m_climbing = false;

};