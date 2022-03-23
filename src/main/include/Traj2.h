#pragma once

#include <cassert>
#include <cmath>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>

#include "Constants.h"

#ifndef PATH_TEST_PROGRAM
#include "SwerveDrive.h"
#endif

struct Cubic3DUnitSpline {
    double p0, v0, p1, v1;
    double t0, t1;

    double at(double t) {
        double dt = t1 - t0;
        double tt = (t - t0) / dt;
        double h00 = (1 + 2 * tt) * (1 - tt) * (1 - tt);
        double h10 = tt * (1 - tt) * (1 - tt);
        double h01 = tt * tt * (3 - 2 * tt);
        double h11 = tt * tt * (tt - 1);

        double a = h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1;
        return a;
    }

    double deriv_at(double t) {
        double dt = t1 - t0;
        double tt = (t - t0) / dt;
        double tt2 = tt * tt;
        double dh00 = 6 * tt2 - 6 * tt;
        double dh10 = 3 * tt2 - 4 * tt + 1;
        double dh01 = -6 * tt2 + 6 * tt;
        double dh11 = 3 * tt2 - 2 * tt;

        double a = dh00 * p0 + dh10 * dt * v0 + dh01 * p1 + dh11 * dt * v1;
        return a;
    }
};

class Traj1D {
   public:
    Traj1D(std::vector<std::tuple<double, double, double>> pts) {
        assert(!pts.empty());
        assert(abs(std::get<2>(pts.at(0))) < 0.001);

        for (size_t i = 0; i + 1 < pts.size(); i++) {
            Cubic3DUnitSpline spline;
            spline.p0 = std::get<0>(pts.at(i));
            spline.v0 = std::get<1>(pts.at(i));
            spline.t0 = std::get<2>(pts.at(i));
            spline.p1 = std::get<0>(pts.at(i + 1));
            spline.v1 = std::get<1>(pts.at(i + 1));
            spline.t1 = std::get<2>(pts.at(i + 1));
            assert(spline.t1 > spline.t0);
            splines.push_back(spline);
        }
    }

    void start(double time) {
        spline_idx = 0;
        time_offset = time;
        started = true;
    }

    std::pair<double, double> continue_get_pos_vel(double time) {
        assert(started);
        time -= time_offset;
        if (spline_idx >= splines.size()) {
            auto& spline = splines.back();
            return std::make_pair(spline.p1, spline.v1);
        }
        auto* spline = &splines[spline_idx];
        while (time > spline->t1) {
            spline_idx += 1;
            if (spline_idx >= splines.size()) {
                return continue_get_pos_vel(9999999999.9);
            }
            spline = &splines[spline_idx];
        }
        return std::make_pair(spline->at(time), spline->deriv_at(time));
    }

    bool is_done(double time) {
        return started && spline_idx >= splines.size();
    }

   private:
    std::vector<Cubic3DUnitSpline> splines;
    size_t spline_idx = 0;
    double time_offset{0.0};
    bool started = false;
};

class RobotTraj {
   public:
    RobotTraj(Traj1D xx, Traj1D yy, Traj1D tt)
        : x{std::move(xx)}, y{std::move(yy)}, theta{std::move(tt)} {
        // Perform some basic verification of the path
        double faketime = 0.0;
        x.start(faketime);
        y.start(faketime);
        theta.start(faketime);

        double r =
            0.5 * 0.0254 * hypot(DriveConstants::Width, DriveConstants::Length);

        while (!(x.is_done(faketime) && y.is_done(faketime) &&
                 theta.is_done(faketime))) {
            double xvel = x.continue_get_pos_vel(faketime).second;
            double yvel = y.continue_get_pos_vel(faketime).second;
            double omega = theta.continue_get_pos_vel(faketime).second;

            double trans_vel = sqrt(xvel * xvel + yvel * yvel);

            double tangential_vel = abs(omega) * r;

            double max_possible_vel = trans_vel + tangential_vel;
            if (max_possible_vel >
                DriveConstants::MAX_TRAJ_WHEEL_VELOCITY_MPS) {
                std::cout << "failure at time " << faketime << std::endl;
                assert(max_possible_vel <
                       DriveConstants::MAX_TRAJ_WHEEL_VELOCITY_MPS);
            }
            faketime += 0.1;
        }

        // Reset after verification
        x.start(0.0);
        y.start(0.0);
        theta.start(0.0);
    }

    void start(double time) {
        x.start(time);
        y.start(time);
        theta.start(time);
    }

#ifndef PATH_TEST_PROGRAM
    void drive(double time, SwerveDrive* sd) {
        auto xd = x.continue_get_pos_vel(time);
        auto yd = y.continue_get_pos_vel(time);
        auto td = theta.continue_get_pos_vel(time);

        sd->TrajectoryFollow2(xd.first, yd.first, td.first, xd.second,
                              yd.second, td.second);
    }
#endif
    bool is_done(double time) {
        return x.is_done(time) && y.is_done(time) && theta.is_done(time);
    }

    Traj1D x;
    Traj1D y;
    Traj1D theta;
};
