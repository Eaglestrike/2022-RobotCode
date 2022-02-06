#include <test/PhysicsSim.h>

double PhysicsSim::SimProfile::GetPeriod() {
    // set the start time if not yet running
    if (!_running) {
        _lastTime = std::chrono::steady_clock::now();
        _running = true;
    }

    // get time since last call
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed(now - _lastTime);
    double const period = elapsed.count();
    _lastTime = now;

    return period;
}