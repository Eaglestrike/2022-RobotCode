#include <map>
#include <vector>
#include <photonlib/PhotonCamera.h>

#include "Limelight.h"

class AutoAimer { 
    public: 
        struct Settings { //in same units of velocity as talons
            double flywheel_speed;
            double hood_pose;
            double kicker_speed;
        };

        AutoAimer();
        AutoAimer::Settings DistanceToSettings(double dist); //writing this in hopes of a range finder :3       

    private: //debating whether or not to put these in constants.h
        std::map<double, AutoAimer::Settings> dist_to_settings;
        std::vector<double> dists;

        bool binary_search(std::vector<double> &array, double p, double &p1, double &p2); //basically the interpolate function from last time
        double interpolate(double dist, double prev_setting, double next_setting, double prev_dist, double next_dist);
};