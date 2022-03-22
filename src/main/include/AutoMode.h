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
        std::vector<double> times2 = {3.0, 3.1, 6.4, 6.5};
        std::vector<int> waypointIndex2 = {0, 0, 0, 0};

        // 3 ball Auto
        std::vector<State> actions3 = {DRIVEnINTAKE, IDLE, SHOOT, IDLE, DRIVE, IDLE, DRIVEnINTAKE, IDLE, DRIVE, IDLE, SHOOT, IDLE, DRIVE, IDLE};
        std::vector<double> times3 = {2.4, 2.5, 4.4, 4.5, 5.2, 5.3, 7.4, 7.5, 8.0, 8.1, 9.9, 10.0, 11.0, 11.1};
        std::vector<int> waypointIndex3 = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4};

        // 5 ball Auto
        // std::vector<State> actions = {SHOOT, IDLE, DRIVEnINTAKE, IDLE, DRIVE, IDLE, DRIVEnINTAKE, IDLE, DRIVE, IDLE, SHOOT, IDLE, DRIVEnINTAKE, IDLE, INTAKE, IDLE, DRIVE, IDLE, SHOOT, IDLE};
        // std::vector<double> times = {0.8, 0.9, 3.0, 3.1, 3.5, 3.6, 6.5, 6.6, 7.5, 7.6, 9.0, 9.1, 11.5, 11.6, 12.0, 12.1, 13.5, 13.6, 14.9, 15.0};
        // std::vector<int> waypointIndex = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5};


        void ResetAuto();
        void Periodic(double time);
        State getState();
        int getWaypointIndex();

    private:
        State state;
        size_t index;
};