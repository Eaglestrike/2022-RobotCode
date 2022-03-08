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
#include <rev/ColorMatch.h>


class Shooter{
    public:
        enum State{
            IDLE,
            SHOOT,
            ZERO,
            AIM,
            LOAD,
            CLIMB,
            MANUAL,
            BadIdea
        };

        Shooter();
        ~Shooter();
        void DisableMotors();
        void Periodic();
        void Aim();
        void Shoot();
        bool withinRange(std::vector<double>& array, double p, double& p1, double& p2);
        void Zero();
        void Load();
        void Stop();
        void setState(State newState);
        bool Aimed();
        void Calibrate();
        void setPID();
        void Manual(double input);
        double convertToRPM(double ticks);
        void setColor(bool isblue);
        void LowShot();
        void peekTurret();

    private:
        //TalonFX in ticks - 0 - 20,000
        //in rpm - 0 -6000

        WPI_TalonFX m_flywheelMaster{ShooterConstants::shootMotorPortMaster, "rio"};
        WPI_TalonFX m_flywheelSlave{ShooterConstants::shootMotorPortSlave, "rio"};
        WPI_TalonFX m_turret{ShooterConstants::turretMotorPort, "rio"};
        WPI_TalonFX m_hood{ShooterConstants::hoodMotorPort, "rio"};
        WPI_TalonFX m_kicker{ShooterConstants::kickerMotorPort, "rio"};

        frc::DigitalInput m_turretLimitSwitch{ShooterConstants::turretLimitSwitch};
        frc::DigitalInput m_photogate{ShooterConstants::photogate};

        frc2::PIDController m_turretController{ShooterConstants::turretP,
            ShooterConstants::turretI, ShooterConstants::turretD};
        
        frc2::PIDController m_turretPos{0, 0, 0};

        State m_state;

        Limelight * m_limelight = new Limelight();

        Channel m_channel;

        double m_angle;
        double m_speed;
        double m_turrPos;

        bool m_hoodZero = false;
        bool m_turretZero = false;

        //For storing hood and angle values for shooting
        //angle , speed
        std::vector<double> dataPoints = {
            -24.0, -23.5, -22.5, -21.5, -20.0, -19.2, -18.0, -16.7, -15.4, 
            -14.5, -12.6, -11.2, -9.7, -8.0, -6.2, -5.0, -2.5, -0.5, 5.5, 
            9.1, 12.4, 16.5, 18.7
        };
        std::unordered_map <double, std::pair<double, double>> dataMap;

        frc::Color ballColor;
        frc::Color redBall{0.57, 0.31, 0.105};
        frc::Color blueBall{0.1536, 0.4, 0.4451};
        frc::Color defualtColor{0.265, 0.45, 0.28};

        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 m_colorSensor{i2cPort};
        rev::ColorMatch m_colorMatcher;
        // https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/C%2B%2B/Read%20RGB%20Values/src/main/cpp/Robot.cpp

        bool m_blue;
        double confidence = 0.65;

        double angle_scale_factor = 1;
        double speed_scale_factor = 1;
};