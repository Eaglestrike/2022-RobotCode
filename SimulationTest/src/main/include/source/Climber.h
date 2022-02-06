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
        void Periodic(double pitch); //executes state actions

        //returns next state of climber
        State Climb();
        State Initialize();
        State Idle();

        void setState(State newState); //can set state manually

        void Calibrate();


    private:
        State state;

        WPI_TalonFX gearboxMaster{ClimbConstants::gearboxPort1};
        WPI_TalonFX gearboxSlave{ClimbConstants::gearboxPort2};

        double nextPitch(double currPitch, double time);

        // //Higher pneumatic
        // frc::Solenoid climbStage1{frc::PneumaticsModuleType::REVPH, 
        //     ClimbConstants::solenoid1Port};
        // //Lower pneumatic
        // frc::Solenoid climbStage2{frc::PneumaticsModuleType::REVPH,
        //     ClimbConstants::solenoid2Port};
};