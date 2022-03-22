#include <Odometry.h>

// Constructor
Odometry::Odometry() {}

// Updates the Odometry Robot location
void Odometry::updateOdometry(double br_angle_rad, double br_dist_m,
                              double bl_angle_rad, double bl_dist_m,
                              double fr_angle_rad, double fr_dist_m,
                              double fl_angle_rad, double fl_dist_m,
                              double theta_rad, double dt) {
    if (!init) {
        prev_bl = bl_dist_m;
        prev_br = br_dist_m;
        prev_fr = fr_dist_m;
        prev_fl = fl_dist_m;
        init = true;
        return;
    }
    double delta_bl_dist_m = bl_dist_m - prev_bl;
    prev_bl = bl_dist_m;
    double delta_br_dist_m = br_dist_m - prev_br;
    prev_br = br_dist_m;
    double delta_fr_dist_m = fr_dist_m - prev_fr;
    prev_fr = fr_dist_m;
    double delta_fl_dist_m = fl_dist_m - prev_fl;
    prev_fl = fl_dist_m;

    double br_y = sin(br_angle_rad) * delta_br_dist_m;
    double br_x = cos(br_angle_rad) * delta_br_dist_m;
    double bl_y = sin(bl_angle_rad) * delta_bl_dist_m;
    double bl_x = cos(bl_angle_rad) * delta_bl_dist_m;
    double fr_y = sin(fr_angle_rad) * delta_fr_dist_m;
    double fr_x = cos(fr_angle_rad) * delta_fr_dist_m;
    double fl_y = sin(fl_angle_rad) * delta_fl_dist_m;
    double fl_x = cos(fl_angle_rad) * delta_fl_dist_m;

    double robot_local_dy = (fl_y + fr_y + bl_y + br_y) / 4.0;
    double robot_local_dx = (fl_x + fr_x + bl_x + br_x) / 4.0;

    double global_dx =
        robot_local_dx * cos(theta_rad) - robot_local_dy * sin(theta_rad);
    double global_dy =
        robot_local_dx * sin(theta_rad) + robot_local_dy * cos(theta_rad);
    // frc::SmartDashboard::PutNumber("m_FWD", m_FWD_new);
    // frc::SmartDashboard::PutNumber("m_STR", m_STR_new);

    m_xdot = global_dx / dt;
    m_ydot = global_dy / dt;

    // From Velocity to Position
    m_x += global_dx;
    m_y += global_dy;
}

// Return X position
double Odometry::getX() { return m_x; }

// Return Y position
double Odometry::getY() { return m_y; }

double Odometry::getYSpeed() { return m_ydot; }

double Odometry::getXSpeed() { return m_xdot; }

// Reset Odometry, without resetting seen encoder values.
// To renormalize to seen encoder values, call updateOdometry,
// then Reset.
void Odometry::Reset() {
    m_x = 0;
    m_y = 0;
    m_xdot = 0;
    m_ydot = 0;
}
