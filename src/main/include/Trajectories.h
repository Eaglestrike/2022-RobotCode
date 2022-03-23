#pragma once

#include "Traj2.h"

namespace trajectories {

std::unique_ptr<RobotTraj> twoball_segment1() {
    // Each axis gets a list of (pos, vel, time) tuples to define its
    // trajectory
    // (dunno if any of these values are right, but hopefully you get the
    // idea)
    auto seg1x = Traj1D({
        std::make_tuple(0.0, 0.0, 0.0),
        std::make_tuple(-0.5, -0.5, 1.5),
        std::make_tuple(-1.0, 0.0, 3.0),
        // hold the end, the traj runs for the longest of the three:
        std::make_tuple(-1.0, 0.0, 3.5),
    });
    auto seg1y = Traj1D({
        std::make_tuple(0.0, 0.0, 0.0),
        std::make_tuple(-0.5, 0.0, 1.5),
        std::make_tuple(-1.0, 0.0, 3.0),
    });
    auto seg1t = Traj1D({
        std::make_tuple(0.0, 0.0, 0.0),
        std::make_tuple(M_PI, 0.0, 1.0),
    });
    return std::make_unique<RobotTraj>(seg1x, seg1y, seg1t);
}

}  // namespace trajectories
