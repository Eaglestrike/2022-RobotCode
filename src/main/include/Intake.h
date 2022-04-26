/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Intake Header File

  For the Intake Subsystem
  Intialize intake motors and pnuematics
  DIO photogates are used to tell where and when we can run the motors

  Now incorporates the channel
*/


#pragma once

#include "Constants.h"
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <FramePeriod.h>
#include <frc/DigitalInput.h>
#include <iostream>

class Intake{
    public:
        enum State {
            IDLE,
            INTAKE,
            RUN,
            TOGGLE,
            UNJAM,
            CLIMB,
            STOP,
            ZERO
        };

        Intake();
        void Periodic();
        void toggle();
        void Deploy();
        void Run();
        void Stop();
        void Retract();
        void Unjam();
        void setState(State newstate);

    private:
        WPI_TalonFX m_intakeMotor{IntakeConstants::intakeMotorPort, "rio"};

        frc::Solenoid pneumatics{frc::PneumaticsModuleType::CTREPCM, 
            IntakeConstants::solenoidPort};

        units::second_t duration{0.1};

        State m_state;
};