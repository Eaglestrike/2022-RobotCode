#pragma once

#include "Constants.h"
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/ColorSensorV3.h>
#include "rev/ColorMatch.h"
#include <frc/I2C.h>

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

        rev::ColorSensorV3 colorSensor{frc::I2C::Port::kOnboard};
        rev::ColorMatch colorMatcher;

        rev::ColorSensorV3 colorSensorKicker{frc::I2C::Port::kMXP}; //the more useless one
        rev::ColorMatch kickerColorMatcher;

        frc::Color red{0.561, 0.232, 0.114}; //red
        frc::Color blue{0.143, 0.427, 0.429}; //blue

        //frc::Solenoid pneumatics{frc::PneumaticsModuleType::REVPH, 
            //IntakeConstants::solenoidPort};

        State state = IDLE;
};