#pragma once

#include "Constants.h"
#include <iostream>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
        void Periodic(double delta_pitch, double pitch, double time, bool passIdle, bool drivenForward, bool retryFirstClimb, bool passDiagonalArmRaise, 
            bool doSecondClimb); //executes state actions

        //returns next state of climber
        State Idle(bool passIdle);
        
        State VerticalArmExtend(bool drivenForward);
        State VerticalArmRetract(double pitch, double delta_pitch, bool retry);

        State TestDiagonalArmExtend();
        State DiagonalArmExtend(double pitch, double delta_pitch, bool passDiagonalArmRaise);
        State DiagonalArmRaise(bool passDiagonalArmRaise);
        State DiagonalArmRetract(bool doSecondClimb, double pitch, double delta_pitch);

        void SetState(State newState); //can set state manually
        State& GetState() {return state;}

        void Calibrate();
        void ToggleStopped() {stopped = !stopped;}
        bool getStopped() {return stopped;}

        //the below public functions are for testing
        void setTime(units::second_t time) {currTime = time.value();}

        bool getFullExtendPneumatic() {return climbFullExtend.Get();}
        bool getMedExtendPneumatic() {return climbMedExtend.Get();}
        double getMasterMotorOutput() {return gearboxMaster.GetMotorOutputPercent(); }
        double getSlaveMotorOutput() {return gearboxSlave.GetMotorOutputPercent(); }

        void InitializeTests();
        void Stop();
        void extendArmUntilStopped(bool inverted);
        void retractArm();
        void setArmLowered();
        void setArmRaised();
        void setArmVertical();

        void testRaiseVerticalArm();
        void testRetractVerticalArm();
        void testSeesIfHooked();
        void testDiagonalExtension();
        void testDiagonalArmRaise();
        void testBarTraversalFromRaised(); //should do correct function even if not actually hanging
        bool stateJustChanged();
        //getters and setters for simulation
        WPI_TalonFX& getMotor() {return gearboxMaster;}
        frc2::PIDController getPIDCntrl() {return motorPIDController;}
        //frc::ProfiledPIDController<units::meters> getProfiledPID() {return motorProfiledPID; }
        frc::Solenoid& getFullExtend() {return climbFullExtend;}
        frc::Solenoid& getMedExtend() {return climbMedExtend;}
        frc::Solenoid& getBrake() {return brake;}


    private:
        State state = IDLE;
        State prevState = IDLE; //for state just changed

        bool stopped = false;
        bool goingTraversal = false;

        double currTime = 0;
        double waitStartTime = 0;
        double pitchBad = false;

        bool hooked();
        bool waited(double time, double startTime);
        bool motorDone(double pose);
        bool pitchGood(double pitch, double delta_pitch);
        bool pitchVeryBad(double pitch, double delta_pitch);
<<<<<<< HEAD
=======
        bool stateJustChanged();
        units::length::meter_t meterPose();
>>>>>>> 153137d7462779dbe95f5a48ef39b3f82962bbdb

        WPI_TalonFX gearboxMaster{ClimbConstants::gearboxPort1};
        WPI_TalonFX gearboxSlave{ClimbConstants::gearboxPort2};

        //want to be fairly slow...
         frc2::PIDController motorPIDController{ClimbConstants::motorP,
            ClimbConstants::motorI, ClimbConstants::motorD};
        // frc::TrapezoidProfile<units::meters>::Constraints constraints{ClimbConstants::motorMaxVel, ClimbConstants::motorMaxAcc};
        // frc::ProfiledPIDController<units::meters> motorProfiledPID{ClimbConstants::motorP, ClimbConstants::motorI, ClimbConstants::motorD, constraints};

        //Higher pneumatic
        frc::Solenoid climbFullExtend{frc::PneumaticsModuleType::REVPH, 
            ClimbConstants::solenoid1Port};
        //Lower pneumatic
        frc::Solenoid climbMedExtend{frc::PneumaticsModuleType::REVPH,
            ClimbConstants::solenoid2Port};

        frc::Solenoid brake{frc::PneumaticsModuleType::REVPH, ClimbConstants::BrakeSolenoidPort};



};