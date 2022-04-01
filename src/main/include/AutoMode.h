#include <iostream>
#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>
// #include "Intake.h"
// #include "Shooter.h"
// #include "SwerveDrive.h"

class AutoMode{
    public:
        AutoMode();

        void SetMode(int autoMode);

        enum State{
            SHOOT,
            INTAKE,
            DRIVE,
            IDLE,
            DRIVEnINTAKE
        };

        std::vector<State> actions;
        std::vector<double> times;
        std::vector<int> waypointIndex;

        //Default Back up and shoot
        std::vector<State> actions1 = {DRIVE, IDLE, SHOOT, IDLE};
        std::vector<double> times1 = {3.0, 3.1, 6.0, 6.1};
        std::vector<int> waypointIndex1 = {0, 0, 0, 0};

        // 2 ball Auto
        std::vector<State> actions2 = {DRIVEnINTAKE, IDLE, SHOOT, IDLE};
        std::vector<double> times2 = {3.0, 3.1, 10.0, 10.1};
        std::vector<int> waypointIndex2 = {0, 0, 0, 0};

        // 3 ball Auto
        std::vector<State> actions3 = {DRIVEnINTAKE, IDLE, SHOOT, IDLE, DRIVE, IDLE, DRIVEnINTAKE, IDLE, DRIVE, IDLE, SHOOT, IDLE, DRIVE, IDLE};
        std::vector<double> times3 = {2.4, 2.5, 5.0, 5.1, 5.7, 5.8, 8.0, 8.1, 8.5, 8.6, 12.0, 12.1, 12.4, 12.5};
        std::vector<int> waypointIndex3 = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 3, 3};

        // 5 ball Auto
        std::vector<State> actions5 = {SHOOT, IDLE, DRIVEnINTAKE, IDLE, DRIVE, IDLE, DRIVEnINTAKE, IDLE, DRIVE, IDLE, SHOOT, IDLE, DRIVEnINTAKE, 
            IDLE, DRIVEnINTAKE, IDLE, SHOOT, IDLE};
        std::vector<double> times5 = {1.0, 1.1, 2.5, 2.6, 3.2, 3.3, 6.0, 6.1, 6.6, 6.7, 7.8, 7.9, 10.2, 
            10.3, 11.7, 11.8, 14.9, 15.0};
        std::vector<int> waypointIndex5 = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4, 5, 5, 5, 5};


        void ResetAuto();
        void Periodic(double time);
        State getState();
        int getWaypointIndex();

    private:
        State state;
        size_t index;
};