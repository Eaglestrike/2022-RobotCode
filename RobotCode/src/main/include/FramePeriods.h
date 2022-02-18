#include <ctre/Phoenix.h>
#include <vector>

//a class. in constructor add talons to it, then call the peridoic function in robot peroidic

class FramePeriods {
    public:
        FramePeriods();
        void addTalon(WPI_TalonFX& talon);
        void periodic();

    private:
        std::vector<WPI_TalonFX*> talons; 

        const int lt = 255; //long time interval
        const int st = 20; //short time interval
        const int statusFrameAttempts = 3; //number of tries to set frame period

        void checkFramePeriods(WPI_TalonFX* talon);
        void setFramePeriods(WPI_TalonFX& talon);


};
