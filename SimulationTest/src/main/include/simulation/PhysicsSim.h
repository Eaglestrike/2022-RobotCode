#pragma once

#include <vector>
#include <iostream> 
#include "ctre/Phoenix.h"

class PhysicsSim
{
    class SimProfile;
    class TalonFXSimProfile;
    
public:
    static PhysicsSim& GetInstance();
    ~PhysicsSim();
    void AddTalonFX(TalonFX &talon, double const accelToFullTime, double const fullVel, bool const sensorPhase = false);
    void Run();

private:
    std::vector<SimProfile*> _simProfiles; //list of sim profiles (like simulated talonfx)

    /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
    static double random(double min, double max);
    static double random(double max);

    class SimProfile {
    private:
        std::chrono::steady_clock::time_point _lastTime;
        bool _running = false;

    public:
        virtual void Run() {}
        virtual ~SimProfile() {}

    protected:
        double GetPeriod();
    };


    class TalonFXSimProfile : public SimProfile {

    private:
        TalonFX &_talon;
        double const _accelToFullTime = 0;
        double const _fullVel = 0;
        bool const _sensorPhase = 0;

        double _pos = 0;
        double _vel = 0;

     public:

        TalonFXSimProfile(TalonFX &talon, double const accelToFullTime, double const fullVel, bool const sensorPhase);


        ~TalonFXSimProfile() override {}

         void Run() override;
    };
};