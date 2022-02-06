#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>



class Climber{
    public:
        enum State{ //feel free to change the names if they suck
            IDLE,

           //only for climb to first bar
            FIRST_ARM_EXTEND,
            DRIVE_FORWARD,
            FIRST_ARM_RETRACT,

            //can be used for all subsequent bars
            DIAGONAL_ARM_EXTEND,
            DIAGONAL_ARM_RAISE, //hooks onto bar
            DIAGONAL_ARM_RETRACT //involves retracting & returning to vertical            
        };

        Climber();
        void Periodic(double pitch); //executes state actions

        //returns next state of climber
        State Idle();
        
        State FirstArmExtend();
        State DriveForward();
        State FirstArmRetract();

        State DiagonalArmExtend();
        State DiagonalArmRaise();
        State DiagonalArmRetract();

        void setState(State newState); //can set state manually

        void Calibrate();


    private:
        State state;

        WPI_TalonFX gearboxMaster{ClimbConstants::gearboxPort1};
        WPI_TalonFX gearboxSlave{ClimbConstants::gearboxPort2};

        double nextPitch(double currPitch, double time);

        //Higher pneumatic
        frc::Solenoid climbStage1{frc::PneumaticsModuleType::REVPH, 
            ClimbConstants::solenoid1Port};
        //Lower pneumatic
        frc::Solenoid climbStage2{frc::PneumaticsModuleType::REVPH,
            ClimbConstants::solenoid2Port};

        //pretty sure there should be a double solenoid too...
};