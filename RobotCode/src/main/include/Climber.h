#pragma once

#include "Constants.h"
#include <iostream>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/controller/PIDController.h>

class Climber{
    public:
        enum State{ //feel free to change the names if they suck
            IDLE,

           //only for climb to first bar
            VERTICAL_ARM_EXTEND,
            VERTICAL_ARM_RETRACT,

            TEST_DIAGONAL_ARM_EXTEND,

            //can be used for all subsequent bars
            DIAGONAL_ARM_EXTEND,
            DIAGONAL_ARM_RAISE, //hooks onto bar
            DIAGONAL_ARM_RETRACT //involves retracting & returning to vertical            
        };

        Climber();
        void Periodic(double delta_pitch, double pitch, double time, bool passIdle, bool drivenForward, bool passDiagonalArmRaise, bool doSecondClimb); //executes state actions

        //returns next state of climber
        State Idle(bool passIdle);
        
        State VerticalArmExtend(bool drivenForward);
        State VerticalArmRetract(double pitch, double delta_pitch);

        State TestDiagonalArmExtend();
        State DiagonalArmExtend(double pitch, double delta_pitch);
        State DiagonalArmRaise(bool passDiagonalArmRaise);
        State DiagonalArmRetract(bool doSecondClimb);

        void SetState(State newState); //can set state manually

        void Calibrate();


        //the below public functions are for testing
        void InitializeTests();
        void testRaiseVerticalArm();
        void testRetractVerticalArm();
        void testSeesIfHooked();
        void testDiagonalExtension();
        void testDiagonalArmRaise();
        void testBarTraversalFromRaised();


    private:
        State state = IDLE;
        State prevState = IDLE; //for state just changed

        double currTime = 0;
        double waitStartTime = 0;

        bool hooked();
        bool waited(double time, double startTime);
        bool motorDone(double pose);
        bool pitchGood(double pitch, double delta_pitch);
        bool stateJustChanged();

        WPI_TalonFX gearboxMaster{ClimbConstants::gearboxPort1};
        WPI_TalonFX gearboxSlave{ClimbConstants::gearboxPort2};

        //want to be fairly slow...
         frc2::PIDController motorPIDController{ClimbConstants::motorP,
            ClimbConstants::motorI, ClimbConstants::motorD};

        //Higher pneumatic
        frc::Solenoid climbFullExtend{frc::PneumaticsModuleType::REVPH, 
            ClimbConstants::solenoid1Port};
        //Lower pneumatic
        frc::Solenoid climbMedExtend{frc::PneumaticsModuleType::REVPH,
            ClimbConstants::solenoid2Port};

        frc::Solenoid brake{frc::PneumaticsModuleType::REVPH, ClimbConstants::BrakeSolenoidPort};



};