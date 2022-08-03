#pragma once

#include "Constants.h"
#include "Limelight.h"
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
// #include "Channel.h"
#include <vector>
#include <unordered_map>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include "ShooterCalc.h"
#include <math.h>


class Shooter{
    public:
        enum State{
            IDLE,
            SHOOT,
            CLIMB,
            AIM,
            LOAD,
            SWIVEL,
            MANUAL,
            HOOD
        };

        Shooter(Swerve* s, Limelight* l);
        ~Shooter();
        void Periodic(bool autonomous);
        void Aim();
        void Climb();
        void Swivel();
        void Shoot();
        void Zero();
        void Load();
        void Stop();
        void setState(State newState);
        bool Aimed();
        void Calibrate();
        void setPID();
        void Manual(double input);
        void setColor(bool isblue);
        void LowShot();
        void enablelimelight();
        void EdgeofTarmac();
        void zeroHood();
        void EjectBall();   

        double getTurretAngle() {
            //return m_turret.GetSelectedSensorPosition();
            return frc::InputModulus(m_turret.GetSelectedSensorPosition(), -turret_ticks_per_rev, turret_ticks_per_rev) 
                / turret_ticks_per_rev * 180.0; //SHOULD result in being between -180 and 180 
        }    
        void zeroTurret() {m_turret.SetSelectedSensorPosition(0); } 

    private:

        double turret_ticks_per_rev = 31000; //TODO: find out

        void TurretAim(double offset);
        //TalonFX in ticks - 0 - 20,000
        //in rpm - 0 -6000
        
        Swerve* swerve; 

        WPI_TalonFX m_flywheelMaster{ShooterConstants::shootMotorPortMaster, "rio"};
        WPI_TalonFX m_flywheelSlave{ShooterConstants::shootMotorPortSlave, "rio"};
        WPI_TalonFX m_turret{ShooterConstants::turretMotorPort, "rio"};
        WPI_TalonFX m_hood{ShooterConstants::hoodMotorPort, "rio"};
        WPI_TalonFX m_kicker{ShooterConstants::kickerMotorPort, "rio"};

        frc::DigitalInput m_turretLimitSwitch{ShooterConstants::turretLimitSwitch};
        frc::DigitalInput m_photogate{ShooterConstants::photogate};

        frc::DigitalInput m_photogate2{7};

        frc2::PIDController m_turretController{ShooterConstants::turretP,
            ShooterConstants::turretI, ShooterConstants::turretD};

        State m_state;

        Limelight * m_limelight;

        ShooterCalc calc{m_limelight, swerve};
        ShooterCalc::Settings settings; //have a global settings object to be accessed in different places

        // Channel m_channel;

        double m_angle;
        double m_speed;
        double m_time;
        double m_turrPos;

        bool m_hoodZero = false;
        bool m_turretZero = false;

        double m_tarmac_speed = 5000;
        double m_tarmac_angle = 1000;

        frc::Color m_ballColor;
        frc::Color m_redBall{0.4, 0.40, 0.18};
        frc::Color m_blueBall{0.18, 0.4, 0.40};
        frc::Color m_defualtColor{0.265, 0.45, 0.28};  

        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 m_colorSensor{i2cPort};
        rev::ColorMatch m_colorMatcher;
        // https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/C%2B%2B/Read%20RGB%20Values/src/main/cpp/Robot.cpp

        bool m_blue;
        double m_confidence = 0.65;

        double m_angle_scale_factor = 0.98;
        double m_speed_scale_factor = 0.95;

        double m_turnInterval = 90;

        bool m_swivelRight = false;
        bool m_swivelLeft = false;

        bool m_autonomous;

        //field oriented stuff will be done in a different class
        //likely swerve

        double turretPosition;
        double limelightXOff;
        double limelightYOff;
};