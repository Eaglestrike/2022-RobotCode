#include <Trajectory.h>


//Waypoint constructor
Trajectory::Waypoint::Waypoint(double y, double x, double velocity,
    double rotation){
        m_x = x;
        m_y = y;
        m_velocity = velocity;
        m_rotation = rotation;
}


//Trajectory Constructor
Trajectory::Trajectory(){
    index = 0;
}


//Return x position of current waypoint
double
Trajectory::getX(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_x;
}


//Return y position of current waypoint
double 
Trajectory::getY(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_y;
}


//Return velocity of current waypoint
double
Trajectory::getVelocity(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_velocity;
}


//Return rotation of current waypoint
double 
Trajectory::getRotation(int index){
    Trajectory::Waypoint p = m_trajectory.at(index);
    return p.m_rotation;
}


//Add Waypoint to trajectory
void
Trajectory::addWaypoint(Trajectory::Waypoint waypoint){
    m_trajectory.push_back(waypoint);
}


//Return index of Waypoint vector
size_t
Trajectory::getIndex(){
    return index;
}


//Return the length of Waypoint vector
size_t
Trajectory::getLength(){
    return m_trajectory.size();
}


//Increase the index for Waypoint vector
void
Trajectory::Progress(){
    if(index == getLength() -1 ){
        return;
    } else {
    index ++;
    }
}


//Clear the Waypoint vector
void
Trajectory::clearTrajectory(){
    m_trajectory.clear();
    index = 0;
}