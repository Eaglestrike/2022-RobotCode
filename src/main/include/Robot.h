#pragma once

#include <AHRS.h>
#include <Constants.h>
#include <SwerveDrive.h>
#include <Trajectory.h>
#include <WheelDrive.h>
#include <frc/Compressor.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <string>

#include "AutoMode.h"
#include "Climber.h"
#include "Intake.h"
#include "Shooter.h"
#include "cameraserver/CameraServer.h"

frc::Joystick l_joy{OIConstants::l_joy_port};
frc::Joystick r_joy{OIConstants::r_joy_port};
frc::XboxController xbox{OIConstants::O_joy_port};
frc::PowerDistribution PDH{1, frc::PowerDistribution::ModuleType::kRev};
cs::UsbCamera camera;

double m_time = 0;
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

    double GetGyroAngleRad();

   private:
    SwerveDrive m_swerve;
    AutoMode m_auto;
    Intake m_intake;
    Shooter m_shooter;
    Climber m_climber;
    AHRS *navx;

    double out;
    bool m_climbing = false;

    frc::SendableChooser<std::string> m_chooser;
    const std::string blueAlliance = "BLUE";
    const std::string redAlliance = "RED";
    std::string m_autoSelected;
};
