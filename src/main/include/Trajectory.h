#pragma once

#include <vector>

class Trajectory{

    struct Waypoint{
        Waypoint(double x, double y, double veloticty, 
            double rotation);
        double m_x;
        double m_y;
        double m_velocity;
        double m_rotation;
    };
    
    public:
        Trajectory(Waypoint p1);
        double getX(int index);
        double getY(int index);
        double getVelocity(int index);
        double getRotation(int index);

    private:
        std::vector<Waypoint> m_trajectory;
};