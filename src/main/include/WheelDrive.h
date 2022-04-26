/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Swerve Module Header File

  Initialize all Swerve Module hardware
*/


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
        // WheelDrive(int angleMotorPort, int speedMotorPort, 
        //     int encoderPortA, int encoderPortB, int pwmPort);
        WheelDrive(int angleMotorPort, int speedMotorPort,
            int encoderPort);
        void drive(double speed, double angle, double offSet);
        void setPID();
        double normalizeEncoderValue();
        double getVelocity();
        double getAngle(double offset);
        void resetEncoder();
        void initialization(double Offset);
        void Debug();
        void Stop();
    
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

        WPI_CANCoder m_canCoder;
        
        frc2::PIDController pidController{DriveConstants::P , DriveConstants::I, DriveConstants::D};
        frc2::PIDController initializeController{DriveConstants::Pinit,
            DriveConstants::Iinit, DriveConstants::Dinit};
};