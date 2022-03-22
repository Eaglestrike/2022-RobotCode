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


frc::Joystick l_joy{OIConstants::l_joy_port};
frc::Joystick r_joy{OIConstants::r_joy_port};
frc::XboxController xbox{OIConstants::O_joy_port};
frc::PowerDistribution PDH{1, frc::PowerDistribution::ModuleType::kRev};
cs::UsbCamera camera;
 
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

  AutoMode m_auto;

  // Swerve module system
  // SwerveDrive m_swerve;
  // Locations for the swerve drive modules relative to the robot center.
  frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
  frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

  // Creating my kinematics object using the module locations.
  frc::SwerveDriveKinematics<4> m_kinematics{
    m_frontLeftLocation, m_frontRightLocation,
    m_backLeftLocation, m_backRightLocation
  };

  // Underlying swerve module sensors/actuators
  WPI_TalonFX  m_fl_angleMotor{DriveConstants::FLanglePort, "Drivebase"};
  WPI_TalonFX  m_fl_speedMotor{DriveConstants::FLspeedPort, "Drivebase"};
  WPI_CANCoder m_fl_canCoder  {DriveConstants::FLencoder,   "Drivebase"};
  WPI_TalonFX  m_fr_angleMotor{DriveConstants::FRanglePort, "Drivebase"};
  WPI_TalonFX  m_fr_speedMotor{DriveConstants::FRspeedPort, "Drivebase"};
  WPI_CANCoder m_fr_canCoder  {DriveConstants::FRencoder,   "Drivebase"};
  WPI_TalonFX  m_rl_angleMotor{DriveConstants::BLanglePort, "Drivebase"};
  WPI_TalonFX  m_rl_speedMotor{DriveConstants::BLspeedPort, "Drivebase"};
  WPI_CANCoder m_rl_canCoder  {DriveConstants::BLencoder,   "Drivebase"};
  WPI_TalonFX  m_rr_angleMotor{DriveConstants::BRanglePort, "Drivebase"};
  WPI_TalonFX  m_rr_speedMotor{DriveConstants::BRspeedPort, "Drivebase"};
  WPI_CANCoder m_rr_canCoder  {DriveConstants::BRencoder,   "Drivebase"};

  // Swerving PID controllers
  frc2::PIDController m_fl_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};
  frc2::PIDController m_fr_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};
  frc2::PIDController m_rl_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};
  frc2::PIDController m_rr_pid{DriveConstants::P , DriveConstants::I, DriveConstants::D};


  Intake m_intake;
  Shooter m_shooter;
  Climber m_climber;

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