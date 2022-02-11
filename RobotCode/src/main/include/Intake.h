#pragma once

#include "Constants.h"
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Intake{
    public:
        enum State {
            IDLE,
            RUN,
            UNJAM,
            CLIMB
        };

        Intake();
        void Periodic();
        void Deploy();
        void Run();
        void Retract();
        void Unjam();
        void setState(State newstate);
        void calibrate();

    private:
        WPI_TalonFX intakeMotor{IntakeConstants::intakeMotorPort};

        //frc::Solenoid pneumatics{frc::PneumaticsModuleType::REVPH, 
            //IntakeConstants::solenoidPort};

        State state = IDLE;
};