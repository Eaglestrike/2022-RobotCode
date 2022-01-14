#pragma once

#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/PWM.h>

class WheelDrive{
    public:
        // WheelDrive(int angleMotorPort, int speedMotorPort, 
        //     int encoderPortA, int encoderPortB);
        WheelDrive(int angleMotorPort, int speedMotorPort, 
            int encoderPortA, int encoderPortB, int pwmPort);

        void drive(double speed, double angle);
        void setPID();
        double normalizeEncoderValue();
        double getVelocity();
        double getAngle();
        void resetEncoder();
        void initialization();
    
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

        //Also tried DIO
        frc::PWM absEncoder;
        
        frc2::PIDController pidController{0.011,0.0004,0.0001};
};