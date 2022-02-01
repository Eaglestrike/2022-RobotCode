#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>


class Climber{
    public:
        enum State{
            IDLE,
            INITIALIZE,
            CLIMB
        };

        Climber();
        void Periodic();
        void setState(State newState);
        void Climb();
        void Initialize();
        void Idle();
        void Calibrate();

    private:
        State state;

        // WPI_TalonFX gearboxMaster{ClimbConstants::gearboxPort1};
        // WPI_TalonFX gearboxSlave{ClimbConstants::gearboxPort2};

        // //Higher pneumatic
        // frc::Solenoid climbStage1{ClimbConstants::solenoid1Port};
        // //Lower pneumatic
        // frc::Solenoid climbStage2{ClimbConstants::solenoid2Port};
};