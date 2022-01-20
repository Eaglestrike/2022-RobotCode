#include <Trajectory.h>


Trajectory::Waypoint::Waypoint(double x, double y, double velocity,
    double rotation){
        m_x = x;
        m_y = y;
        m_velocity = velocity;
        m_rotation = rotation;
    }


Trajectory::Trajectory(Waypoint p1){
    m_trajectory.push_back(p1);
}


double
Trajectory::getX(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_x;
}


double 
Trajectory::getY(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_y;
}


double
Trajectory::getVelocity(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_velocity;
}


double 
Trajectory::getRotation(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_rotation;
}