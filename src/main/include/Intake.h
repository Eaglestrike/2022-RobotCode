#pragma once

#include "Constants.h"
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <FramePeriod.h>
#include <frc/DigitalInput.h>

class Intake{
    public:
        enum State {
            IDLE,
            RUN,
            UNJAM,
            CLIMB,
            STOP,
            ZERO
        };

        Intake();
        void Periodic();
        void Deploy();
        void Run();
        void Stop();
        void Retract();
        void Unjam();
        void setState(State newstate);

    private:
        WPI_TalonFX m_intakeMotor{IntakeConstants::intakeMotorPort, "Drivebase"};

        frc::Solenoid pneumatics{frc::PneumaticsModuleType::CTREPCM, 
            IntakeConstants::solenoidPort};

        units::second_t duration{0.1};

        State m_state;
};