#include "MoveShoot.h"
using namespace std;



//like 220 from table side
//like 180 from table back

pair<int, int> MoveShoot::distance_to_settings(double dist) {
    double s1, s2, a1, a2, d1, d2;
    for (pair<double, pair<int, int>> p : dist_to_settings) {
        if (p.first >= dist) { //this is upper bound
            d2 = p.first;
            s2 = p.second.second;
            a2 = p.second.first;
            break;
        }
        //lower bound for next loop
        d1 = p.first;
        s1 = p.second.second;
        a1 = p.second.first;
    }

    //interpolate (linear piecewise for now)
    if (a2 == 0 && a1 != 0) a2 = a1; //in case upper bound was never reached
    if (s2 == 0 && s1 != 0) s2 = s1;
    if (d2 == 0 && d1 != 0) d2 = d1;

    if (d1 == d2) return {s1, a1};

    int speed = s1 + (s2 - s1)*((dist-d1)/(d2-d1));
    int angle = a1 + (a2 - a1)*((dist-d1)/(d2-d1));
    
    return {speed, angle};
}

double MoveShoot::distance_to_time(double dist) {
    double t1, t2, d1, d2;
    for (pair<double, double> d : dist_to_time) {
        if (d.first >= dist) {
            d2 = d.first;
            t2 = d.second;
        }
        d1 = d.first;
        t1 = d.second;
    }

    //interpolate (linear piecewise for now)
    if (t2 == 0 && t1 != 0) t2 = t1; //in case upper bound was never reached
    if (d2 == 0 && d1 != 0) d2 = d1;

    if (d1 == d2) return t1;

    return t1 + (t2 - t1)*((dist-d1)/(d2-d1));
}

//could end up becoming deprecated (and just use angle); but I'll put it for now
double MoveShoot::y_offset_to_dist(double y_offset) {
    return ((HUB_HEIGHT - CAM_HEIGHT) / tan ( (CAM_ANGLE + y_offset) * PI / 180.0 ));
}

double MoveShoot::getXVel(double rvx, double rvy) {
    return rvx*sin(limelight.getXOff()*2*PI/180) + rvy*sin((90-limelight.getXOff())*2*PI/180);
}

double MoveShoot::getYVel(double rvx, double rvy) {
    return rvx*cos(limelight.getXOff()*2*PI/180) + rvy*cos((90-limelight.getXOff())*2*PI/180);
}

MoveShoot::MoveShoot(Limelight& l, Swerve& s, frc::Pose2d start) : limelight(l), swerve(s), startPose(start){
    
    dist_to_settings[y_offset_to_dist(-19.4)] = {5700, 17200};
    dist_to_settings[y_offset_to_dist(-18.9)] = {5700, 16400};
    dist_to_settings[y_offset_to_dist(-17.5)] = {5600, 15900};
    dist_to_settings[y_offset_to_dist(-14.8)] = {5600, 14900};
    dist_to_settings[y_offset_to_dist(-12.5)] = {5600, 14000};
    dist_to_settings[y_offset_to_dist(-9.5)] = {5500, 13500};
    dist_to_settings[y_offset_to_dist(-7.4)] = {5400, 13200};
    dist_to_settings[y_offset_to_dist(-5.4)] = {5200, 12600};
    dist_to_settings[y_offset_to_dist(-1.8)] = {4800, 12300};
    dist_to_settings[y_offset_to_dist(0.5)] = {4700, 12200};
    dist_to_settings[y_offset_to_dist(1.7)] = {4500, 12000};
    dist_to_settings[y_offset_to_dist(3.65)] = {4100, 12000};
    dist_to_settings[y_offset_to_dist(6.0)] = {3800, 11800};
    dist_to_settings[y_offset_to_dist(9.9)] = {2800, 11700};
    dist_to_settings[y_offset_to_dist(14.2)] = {2800, 11500};
    dist_to_settings[y_offset_to_dist(16.0)] = {2600, 11000};
    dist_to_settings[y_offset_to_dist(19.88)] = {1800, 10500};

    dist_to_time[y_offset_to_dist(-19.4)] = 1.49;
    dist_to_time[y_offset_to_dist(-18.9)] = 1.4;
    dist_to_time[y_offset_to_dist(-17.5)] = 1.21;
    dist_to_time[y_offset_to_dist(-14.8)] = 1.22;
    dist_to_time[y_offset_to_dist(-12.5)] = 1.1;
    dist_to_time[y_offset_to_dist(-9.5)] = 1;
    dist_to_time[y_offset_to_dist(-7.4)] = 0.89;
    dist_to_time[y_offset_to_dist(-5.4)] = 0.85;
    dist_to_time[y_offset_to_dist(-1.8)] = 0;
    dist_to_time[y_offset_to_dist(0.5)] = 0;
    dist_to_time[y_offset_to_dist(1.7)] = 0;
    dist_to_time[y_offset_to_dist(3.65)] = 0;
    dist_to_time[y_offset_to_dist(6.0)] = 0;
    dist_to_time[y_offset_to_dist(9.9)] = 0;
    dist_to_time[y_offset_to_dist(14.2)] = 0;
    dist_to_time[y_offset_to_dist(16.0)] = 0;
    dist_to_time[y_offset_to_dist(19.88)] = 0;
}

MoveShoot::Settings MoveShoot::calculate() {
    double dist = y_offset_to_dist(limelight.getYOff());
    double time = distance_to_time(dist);

    frc::ChassisSpeeds speeds = swerve.getSpeeds();

    double RVX = getXVel(speeds.vx.value(), speeds.vy.value());
    double RVY = getYVel(speeds.vx.value(), speeds.vy.value());

    frc::SmartDashboard::PutNumber("Robot X velocity", RVX);
    frc::SmartDashboard::PutNumber("Robot Y velocity", RVY);

    pair<double, double> ball_pose = {-RVX*time, dist-RVY*time};
    double a = sqrt((RVX*time*RVX*time) + (RVY*time*RVY*time)); 
    double b = sqrt(ball_pose.first*ball_pose.first + ball_pose.second*ball_pose.second);

    double theta = (RVX < 0)? acos((dist*dist + b*b - a*a) / (2*dist*b)) * 180.0 / PI : -acos((dist*dist + b*b - a*a) / (2*dist*b)) * 180.0 / PI;
    double new_dist = b;

    if (new_dist < 0) {
        cout << "new dist less than zero! ball will always overshoot\n";
        return {NAN, NAN, NAN}; 
    }

    if (new_dist > 6) {
        cout << "new dist less than zero! ball will always overshoot\n";
        return {NAN, NAN, NAN}; 
    }

    pair<double, double> shoot_settings = distance_to_settings(new_dist);
    return {shoot_settings.first, shoot_settings.second, theta};
}

