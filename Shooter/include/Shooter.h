#pragma once

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/Servo.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <photonlib/PhotonCamera.h>
#include <Constants.h>
#include <AutoAimer.h>
#include <Limelight.h>

class Shooter { 
    public: 
        enum State {
            Zeroing,
            Idle,
            AutoAiming,
            ManualAiming,
            Shooting
        };

        Shooter();

        void Periodic(double goal_rot);
        void ManualAim(double goal_rot);
        void AutoAim();
        void Shoot();
        void Zero();

        void setState(State newState);
        bool ReadyToShoot();

    private:
        State state;
        //TODO: replace 0 with actual ids

        //motors
        WPI_TalonFX * turret = new WPI_TalonFX(0);
        WPI_TalonFX * flywheel_master = new WPI_TalonFX(0);
        WPI_TalonFX * flywheel_slave = new WPI_TalonFX(0);
        WPI_TalonFX * hood = new WPI_TalonFX(0);
        WPI_TalonFX * kicker = new WPI_TalonFX(0);

        //limit switch
        frc::DigitalInput * turret_limit_switch = new frc::DigitalInput(1);

        //Camera 
        //TODO: measure cam height & angle (1st & 3rd args) before testing to ensure accuracy 
        Limelight limelight;

        //controllers
        frc2::PIDController turretPID{ShooterConstants::Tp, ShooterConstants::Ti, ShooterConstants::Td};
        frc2::PIDController flywheelPID{ShooterConstants::FWp, ShooterConstants::FWi, ShooterConstants::FWd};
        frc2::PIDController hoodPID{ShooterConstants::Hp, ShooterConstants::Hi, ShooterConstants::Hd};
        AutoAimer auto_aimer;


        
};