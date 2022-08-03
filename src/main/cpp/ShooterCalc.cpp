#include "ShooterCalc.h"
using namespace std;

/**
 * Takes in distance to goal and converts it to speed and angle settings
 * Uses linear piecewise interpolation and data from dist_to_settings map
 * @param dist the horizontal distance to the goal (calculated via limelight)
 * @returns a pair of integers containing the (ideally) best flywheel speed & hood angle for this distance
**/
pair<int, int> ShooterCalc::distance_to_settings(double dist) {
    double s1, s2, a1, a2, d1, d2;
    for (pair<double, pair<int, int>> p : dist_to_settings) {
        if (p.first >= dist) { //this is upper bound (TODO: replace with c++ upper bound function?)
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

/**
 * Similar to the above function. Interpolates the estimated time for the ball to reach the goal
 * Uses same linear interpolation for now, but I'm open to using cuve fitting/physics modeling if we think it'd
 * yield better results
 * @param dist the horizontal distance to the goal (calculated via limelight)
 * @returns the time for the ball to reach the goal
**/
double ShooterCalc::distance_to_time(double dist) {
    double t1, t2, d1, d2; //should always get initialized? should be 0 by default and I don't want to have 4 separate
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

//for odometry: the position of the goal is (0, 0)
//https://www.desmos.com/calculator/bilvmnobux

/**
 * Takes in robot's x and y velocity components (field oriented) and output's the robot's velocity tangential to the goal
 * @param rvx the robot's field oriented x velocity component
 * @param rvy the robot's field oriented y velocity component
 * @param rcoords the roobot's coordinates
 * @returns the robot's velocity tangential to the goal
**/
double ShooterCalc::getXVel(double rvx, double rvy, frc::Pose2d rcoords) {
    //cross product of robot velocity and unit velocity to goal
    double d = sqrt(rcoords.X().value()*rcoords.X().value()* + rcoords.Y().value()*rcoords.Y().value());
    double ux = -rcoords.X().value() / d;
    double uy = -rcoords.Y().value() / d;
    double cross = rvx*ux - rvy*uy;
    return cross;

}

/**
 * Takes in robot's x and y velocity components (field oriented) and output's the robot's velocity toward to the goal
 * @param rvx the robot's field oriented x velocity component
 * @param rvy the robot's field oriented y velocity component
 * @param rcoords the robot's coords
 * @returns the robot's velocity toward to the goal
**/
double ShooterCalc::getYVel(double rvx, double rvy, frc::Pose2d rcoords) {
    //dot product of robot velocity and unit vector to goal
    double d = sqrt(rcoords.X().value()*rcoords.X().value()* + rcoords.Y().value()*rcoords.Y().value());
    double ux = -rcoords.X().value() / d;
    double uy = -rcoords.Y().value() / d;
    double dot = rvx*ux + rvy*uy;
    return dot;
}


//keeping this here for table initialization purposes but I kinda hate this (TODO: redo table with distances?)
//I could move this to the limelight class, but for now I think it's simpler to leave it here
double ShooterCalc::y_offset_to_dist(double y_offset) {
    return ((HUB_HEIGHT - CAM_HEIGHT) / tan ( (CAM_ANGLE + y_offset) * M_PI / 180.0 ));
}


/**
 * The constructor. fills up table
 * @param l the limelight this class is referencing
 * @param s the swerve object this class is referencing
**/
ShooterCalc::ShooterCalc(Limelight* l, Swerve* s) : limelight(l), swerve(s) {
    
    //most recent settings, may need re-tuning
    dist_to_settings[y_offset_to_dist(-20.0)] = {-1800, 12000};
    dist_to_settings[y_offset_to_dist(-18.6)] = {-1700, 11500};
    dist_to_settings[y_offset_to_dist(-17.3)] = {-1690, 11200};
    dist_to_settings[y_offset_to_dist(-16.0)] = {-1600, 10800};
    dist_to_settings[y_offset_to_dist(-14.5)] = {-1500, 10500};
    dist_to_settings[y_offset_to_dist(-13.5)] = {-1200, 9800};
    dist_to_settings[y_offset_to_dist(-11.0)] = {-900, 9500};
    dist_to_settings[y_offset_to_dist(-9.0)] = {-600, 9200};
    dist_to_settings[y_offset_to_dist(-7.0)] = {-450, 9000};
    dist_to_settings[y_offset_to_dist(-5.0)] = {-300, 8800};
    dist_to_settings[y_offset_to_dist(-3.5)] = {-200, 8600};
    dist_to_settings[y_offset_to_dist(-1.0)] = {-100, 8400};
    dist_to_settings[y_offset_to_dist(0.0)] = {-100, 8300};
    dist_to_settings[y_offset_to_dist(3.32)] = {0, 8000};
    dist_to_settings[y_offset_to_dist(4.15)] = {0, 7750};
    dist_to_settings[y_offset_to_dist(6.99)] = {0, 7750};
    dist_to_settings[y_offset_to_dist(10.0)] = {0, 7750};
    dist_to_settings[y_offset_to_dist(20.0)] = {0, 7750};


    //outdated
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

/**
 * Main purpose of this class. Calculates flywheel speed, hood angle, turret angle
 * If the robot velocities are reported as non-zero it will attempt to shoot while moving
 * @returns the calculated settings object
**/
ShooterCalc::Settings ShooterCalc::calculate() {
    double dist = limelight->getDist();
    double time = distance_to_time(dist);

    frc::ChassisSpeeds speeds = swerve->getSpeeds();

    double RVX = getXVel(speeds.vx.value(), speeds.vy.value(), swerve->getPose());
    double RVY = getYVel(speeds.vx.value(), speeds.vy.value(), swerve->getPose());

    frc::SmartDashboard::PutNumber("Robot X velocity", RVX);
    frc::SmartDashboard::PutNumber("Robot Y velocity", RVY);

    pair<double, double> ball_pose = {-RVX*time, dist-RVY*time};
    double a = sqrt((RVX*time*RVX*time) + (RVY*time*RVY*time)); 
    double b = sqrt(ball_pose.first*ball_pose.first + ball_pose.second*ball_pose.second);

    double theta = (RVX < 0)? acos((dist*dist + b*b - a*a) / (2*dist*b)) * 180.0 / M_PI : -acos((dist*dist + b*b - a*a) / (2*dist*b)) * 180.0 / M_PI;
    double new_dist = b;

    if (new_dist < 0) {
        //cout << "new dist less than zero! ball will always overshoot\n";
        return {NAN, NAN, NAN}; 
    }

    //could have a max distance?
    // if (new_dist > 6) {
    //    // cout << "new dist greater than max! ball will always undershoot\n";
    //     return {NAN, NAN, NAN}; 
    // }

    pair<double, double> shoot_settings = distance_to_settings(new_dist);
    return {shoot_settings.first, shoot_settings.second, theta};
}

//previous setting maps    

 // Arizona North Values - Shooter v1
    // dataMap[-24.0] = {5800, 18500};
    // dataMap[-23.5] = {5750, 18500};
    // dataMap[-22.5] = {5700, 18500};
    // dataMap[-21.5] = {5600, 18500};
    // dataMap[-20.0] = {5500, 18200};
    // dataMap[-19.2] = {5500, 18000};
    // dataMap[-18.0] = {5400, 17400};
    // dataMap[-16.7] = {5400, 17000}; 
    // dataMap[-15.4] = {4900, 15700};
    // dataMap[-14.5] = {4600, 15500};
    // dataMap[-12.6] = {4500, 14500};
    // dataMap[-11.2] = {4400, 13800};
    // dataMap[-9.7] = {4400, 13600};
    // dataMap[-8.0] = {4200, 13500};
    // dataMap[-6.2] = {4000, 13500};
    // dataMap[-5.0] = {3600, 14000};
    // dataMap[-2.5] = {3000, 13900};
    // dataMap[-0.5] = {2400, 13400};
    // dataMap[5.5] = {1900, 12900};
    // dataMap[9.1] = {1700, 12900};
    // dataMap[12.4] = {1500, 12800};
    // dataMap[16.5] = {1300, 12700};
    // dataMap[18.7] = {1200, 12500};

    // Monterey Values - Shooter v1
    // m_dataMap[-19.4] = {5700, 17200};
    // m_dataMap[-18.9] = {5700, 16400};
    // m_dataMap[-17.5] = {5600, 15900};
    // m_dataMap[-14.8] = {5600, 14900};
    // m_dataMap[-12.5] = {5600, 14000};
    // m_dataMap[-9.5] = {5500, 13500};
    // m_dataMap[-7.4] = {5200, 13200};
    // m_dataMap[-5.4] = {4800, 12600};
    // m_dataMap[-1.8] = {4400, 12300};
    // m_dataMap[0.5] = {4300, 12200};
    // m_dataMap[1.7] = {4100, 12000};
    // m_dataMap[3.65] = {3800, 12000};
    // m_dataMap[6.0] = {3400, 11800};
    // m_dataMap[9.9] = {2500, 11700};
    // m_dataMap[14.2] = {2300, 11500};
    // m_dataMap[16.0] = {2000, 11000};
    // m_dataMap[19.88] = {1800, 10500};

    // Huoston Championship Values - Shooter v2 flat shots
    // m_dataMap[-18.9] = {-2800, 10500};
    // m_dataMap[-16.5] = {-2500, 10000};
    // m_dataMap[-14.5] = {-1800, 9700};
    // m_dataMap[-13.0] = {-1600, 9600};
    // m_dataMap[-11.3] = {-1400, 9500};
    // m_dataMap[-9.0] = {-1000, 9200};
    // m_dataMap[-5.68] = {-750, 8700};
    // m_dataMap[0.00] = {-600, 8100};
    // m_dataMap[4.8] = {-400, 7750};
    // m_dataMap[8.9] = {-300, 7750};
    // m_dataMap[11.8] = {-200, 7500};
    // m_dataMap[16.25] = {0, 7500};

    // m_dataMap[-20.0] = {-1800, 12000};
    // m_dataMap[-18.6] = {-1700, 11500};
    // m_dataMap[-17.3] = {-1690, 11200};
    // m_dataMap[-16.0] = {-1600, 10800};
    // m_dataMap[-14.5] = {-1500, 10500};
    // m_dataMap[-13.5] = {-1200, 9800};
    // m_dataMap[-11.0] = {-900, 9500};
    // m_dataMap[-9.0] = {-600, 9200};
    // m_dataMap[-7.0] = {-450, 9000};
    // m_dataMap[-5.0] = {-300, 8800};
    // m_dataMap[-3.5] = {-200, 8600};
    // m_dataMap[-1.0] = {-100, 8400};
    // m_dataMap[0.0] = {-100, 8300};
    // m_dataMap[3.32] = {0, 8000};
    // m_dataMap[4.15] = {0, 7750};
    // m_dataMap[6.99] = {0, 7750};
    // m_dataMap[10.0] = {0, 7750};
    // m_dataMap[20.0] = {0, 7750};