#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class Limelight {
    public:
    Limelight();
    double CalculateDistance(); //perhaps not necessary, but may help interpolation be more accurate
    double GetXOff();
    double GetYOff();
    bool SeesTarget();
    void SetLEDMode(std::string mode);
    std::shared_ptr<nt::NetworkTable> GetNetworkTable();

    private:
        void ReadPeriodicIn();
        std::shared_ptr<nt::NetworkTable> network_table;
        std::string table_name = "limelight";

        const int PIPELINE = 0;

        //TODO: measure these on robot for increased accuracy
        const double CAMERA_HEIGHT = 0.4572; //in meters, but i believe we're eschewing wpilib units?
        const double GOAL_HEIGHT = 2.6416; //meters
        const double CAMERA_PITCH = 50; //degrees, from horizontal
};