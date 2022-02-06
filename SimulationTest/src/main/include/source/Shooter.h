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
        double interpolate(double dist, double prev_setting, double next_setting, double prev_dist, double next_dist);
        void Zero();
        void Load();
        void Stop();
        void setState(State newState);
        bool Aimed();
        void Calibrate();
        void setPID();

        WPI_TalonSRX& getFlywheel() {return m_flywheelMaster;}
        WPI_TalonSRX& getTurret() {return m_turret;}

    private:
        WPI_TalonSRX m_flywheelMaster{ShooterConstants::shootMotorPortMaster};
        WPI_TalonSRX m_flywheelSlave{ShooterConstants::shootMotorPortSlave};
        WPI_TalonSRX m_turret{ShooterConstants::turretMotorPort};
        WPI_TalonSRX m_hood{ShooterConstants::hoodMotorPort};
        WPI_TalonSRX m_kicker{ShooterConstants::kickerMotorPort};

        frc::DigitalInput m_turretLimitSwitch{ShooterConstants::turretLimitSwitch};
        // frc::DigitalInput m_photogate{ShooterConstants::photogate};

        frc2::PIDController m_turretController{ShooterConstants::turretP,
            ShooterConstants::turretI, ShooterConstants::turretD};
        frc2::PIDController m_flywheelController{ShooterConstants::flywheelP,
            ShooterConstants::flywheelI, ShooterConstants::flywheelD};
        frc2::PIDController m_hoodController{ShooterConstants::hoodP,
            ShooterConstants::hoodI, ShooterConstants::hoodD};

        State m_state;

        Limelight * m_limelight = new Limelight();

        Channel m_channel;

        double m_angle;
        double m_speed;

        //For storing hood and angle values for shooting
        //angle , speed
        std::vector<double> dataPoints = {
            -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5
        };
        std::unordered_map <double, std::pair<double, double>> dataMap;

};