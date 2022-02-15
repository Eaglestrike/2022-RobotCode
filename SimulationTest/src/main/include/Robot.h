// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <source/Shooter.h>
#include <source/Climber.h>
#include "simulation/testClimbOneBar.h"
#include "simulation/PhysicsSim.h"



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
  Climber climber;
  // frc::Joystick joystick{0};
  // testClimbOneBar test{climber};
  frc::Joystick l_joy{OIConstants::l_joy_port};
  frc::Joystick r_joy{OIConstants::r_joy_port};
  frc::XboxController xbox{OIConstants::O_joy_port};

};
