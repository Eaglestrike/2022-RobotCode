/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Shooter header file
  Incoporates limelight & channel

  *** Channel is dead class
  was removed on the robot as of April 2022 *** 

  Sets flywheel, turret, and hood motors to proper positions
  All shooter values are included
*/


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
#include "Odometry.h"
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
            BadIdea,
            Tarmac,
            Hood
        };

        Shooter();
        ~Shooter();
        void DisableMotors();
        void Periodic(bool autonomous, double navX);
        void Aim();
        void Climb();
        void Swivel();
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
        void peekTurret(double navX, double POV);
        void enablelimelight();
        void EdgeofTarmac();
        void zeroHood();
        void EjectBall();
        void GetOdom(Odometry *odom);
        void MovementOffsetCalculation(double angle);
        double getFieldAng(double navX);
        

    private:
        //TalonFX in ticks - 0 - 20,000
        //in rpm - 0 -6000
        Odometry *m_odom;

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

        Limelight * m_limelight = new Limelight();

        // Channel m_channel;

        double m_angle;
        double m_speed;
        double m_time;
        double m_turrPos;

        bool m_hoodZero = false;
        bool m_turretZero = false;

        double m_tarmac_speed = 5000;
        double m_tarmac_angle = 1000;

        // Arizona North Values
        // std::vector<double> dataPoints = {
        //     -24.0, -23.5, -22.5, -21.5, -20.0, -19.2, -18.0, -16.7, -15.4, 
        //     -14.5, -12.6, -11.2, -9.7, -8.0, -6.2, -5.0, -2.5, -0.5, 5.5, 
        //     9.1, 12.4, 16.5, 18.7
        // };

        // Monterey Values
        // std::vector<double> m_dataPoints = {-19.4, -18.9, -17.5, -14.8, -12.5, -9.5,
        //     -7.4, -5.4, -1.8, 0.5, 1.7, 3.65, 6.0, 9.9, 14.2, 16.0, 19.88};

        // Initial New Shooter Values
        //flat shots
        // std::vector<double> m_dataPoints = {-18.9, -16.5, -14.5, -13.0, -11.3, -9.0, -5.68, 0.0, 4.8, 8.9, 11.8, 16.25};
        
        // Huoston Championship Values
        std::vector<double> m_dataPoints = {-20.0, -18.6, -17.3, -16.0, -14.5, -13.5, -11.0, -9.0, -7.0, -5.0, -3.5,
            -1.0, 0.0, 3.32, 4.15, 6.99, 10.0, 20.0};
        std::unordered_map <double, std::pair<double, double>> m_dataMap;

        // For limelight and time values
        std::vector<std::pair<double, double>> m_timeMap;

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

        // x, y magnitudes for goal offset in feild orient
        std::pair <double, double> m_goalOffset;
        double turnTheta = 0.0;
        double thetaOff;
        double dOff;
        double dOffConverted;

        double turretPosition;
        double limelightXOff;
        double limelightYOff;
};