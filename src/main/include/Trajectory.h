#pragma once

#include <vector>

class Trajectory{
    
    public:
        struct Waypoint{
        Waypoint(double y, double x, double veloticty, 
            double rotation);
        double m_x;
        double m_y;
        double m_velocity;
        double m_rotation;
        };

        Trajectory();
        void addWaypoint(Trajectory::Waypoint waypoint);
        double getX(int index);
        double getY(int index);
        double getVelocity(int index);
        double getRotation(int index);
        size_t getIndex();
        size_t getLength();
        void Progress();
        void clearTrajectory();

    private:
        std::vector<Waypoint> m_trajectory;

        size_t index = 0;
};