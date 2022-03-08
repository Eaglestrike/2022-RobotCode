#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>


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

        //Manual Climb stuff
        void armExtension(double input);
        void ExtendfirstStage();
        void ExtendsecondStage();
        void RetractfirstStage();
        void RetractsecondStage();
        void enableBrake();
        void whenDisabled();


    private:
        State state;

        WPI_TalonFX gearboxMaster{ClimbConstants::gearboxPort1, "rio"};
        WPI_TalonFX gearboxSlave{ClimbConstants::gearboxPort2, "rio"};

        //Higher pneumatic
        frc::Solenoid climbStage1{frc::PneumaticsModuleType::REVPH, 
            ClimbConstants::solenoid1Port};
        //Lower pneumatic
        frc::Solenoid climbStage2{frc::PneumaticsModuleType::REVPH,
            ClimbConstants::solenoid2Port};

        //Disk Brake
        frc::Solenoid diskBrake{frc::PneumaticsModuleType::REVPH,
            ClimbConstants::diskBrakePort};

        units::time::second_t pulse{0.1};
};