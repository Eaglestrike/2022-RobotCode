#pragma once

#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PWM.h>
#include <units/angle.h>
#include <math.h>
#include <Constants.h>

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
        static constexpr double MAX_VOLTS = DriveConstants::MAX_VOLTS;
        
        int m_prevEncoderValue = 0;
        int m_currEncoderValue = 0;
        double m_angle = 0;
        bool m_reverse = false;
        double m_speedOut = 0;

        double speedGearRatio = DriveConstants::speedGearRatio;
        double angleGearRatio = DriveConstants::angleGearRatio;

        WPI_TalonFX angleMotor;
        WPI_TalonFX speedMotor;
        
        frc::Encoder encoder;

        frc::DutyCycleEncoder absEncoder;
        
        frc2::PIDController pidController{DriveConstants::P , DriveConstants::I, DriveConstants::D};
        frc2::PIDController initializeController{DriveConstants::Pinit,
            DriveConstants::Iinit, DriveConstants::Dinit};
};