#pragma once

#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PWM.h>
#include <units/angle.h>
#include <math.h>

#define PI 3.14159265

class WheelDrive{
    public:
        WheelDrive(int angleMotorPort, int speedMotorPort, 
            int encoderPortA, int encoderPortB, int pwmPort);
        void drive(double speed, double angle);
        void setPID();
        double normalizeEncoderValue();
        double getVelocity();
        double getAngle();
        void resetEncoder();
        void initialization(double Offset);
        void Debug();
        double GetabsAngle();
    
    private:
        static constexpr double MAX_VOLTS = 1.0;
        int m_prevEncoderValue = 0;
        int m_currEncoderValue = 0;
        double m_angle = 0;
        bool m_reverse = false;
        double m_speedOut = 0;

        double speedGearRatio = 1/6.12;
        double angleGearRatio = 1/12.8;

        WPI_TalonFX angleMotor;
        WPI_TalonFX speedMotor;
        
        frc::Encoder encoder;

        frc::DutyCycleEncoder absEncoder;
        
        frc2::PIDController pidController{0.012,0.008,0.0001};
        frc2::PIDController initializeController{4.8,1.2,0.02};
};