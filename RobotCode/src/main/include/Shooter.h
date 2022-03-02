#pragma once

#include "Constants.h"
#include "Limelight.h"
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Channel.h"
#include <vector>
#include <unordered_map>
#include <rev/ColorSensorV3.h>
#include "rev/ColorMatch.h"


class Shooter{
    public:
        enum State{
            IDLE,
            SHOOT,
            ZERO,
            AIM,
            LOAD,
            MANUAL
        };

        Shooter();
        void Periodic();
        void Aim();
        void Shoot();
        bool withinRange(std::vector<double> array, double p, double p1, double p2);
        void Zero();
        void Load();
        void Stop();
        void setState(State newState);
        bool Aimed();
        void Calibrate();
        void setPID();
        void Manual(double input);
        void limelightOFF();

    private:
        WPI_TalonFX m_flywheelMaster{ShooterConstants::shootMotorPortMaster};
        WPI_TalonFX m_flywheelSlave{ShooterConstants::shootMotorPortSlave};
        WPI_TalonFX m_turret{ShooterConstants::turretMotorPort};
        WPI_TalonFX m_hood{ShooterConstants::hoodMotorPort};
        WPI_TalonFX m_kicker{ShooterConstants::kickerMotorPort};

        frc::DigitalInput m_turretLimitSwitch{ShooterConstants::turretLimitSwitch};
        frc::DigitalInput m_photogate{ShooterConstants::photogate};

        rev::ColorSensorV3 colorSensor{frc::I2C::Port::kMXP}; //the more useless one
        rev::ColorMatch colorMatcher;
        frc::Color red{0.561, 0.232, 0.114}; //red
        frc::Color blue{0.143, 0.427, 0.429}; //blue

        frc2::PIDController m_turretController{ShooterConstants::turretP,
            ShooterConstants::turretI, ShooterConstants::turretD};
        frc2::PIDController m_flywheelController{ShooterConstants::flywheelP,
            ShooterConstants::flywheelI, ShooterConstants::flywheelD};
        frc2::PIDController m_hoodController{ShooterConstants::hoodP,
            ShooterConstants::hoodI, ShooterConstants::hoodD};

        State m_state = IDLE;

        Limelight * m_limelight = new Limelight();

        Channel m_channel;

        double m_angle;
        double m_speed;

        bool m_hoodZero = false;
        bool m_turretZero = false;

        //For storing hood and angle values for shooting
        //angle , speed
        std::vector<double> dataPoints = {
            -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5
        };
        std::unordered_map <double, std::pair<double, double>> dataMap;

};