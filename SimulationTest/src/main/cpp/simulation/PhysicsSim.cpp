#include <simulation/PhysicsSim.h>

PhysicsSim& PhysicsSim::GetInstance() {
    static PhysicsSim sim;
    return sim;
}

PhysicsSim::~PhysicsSim() {
    _simProfiles.clear();
}

void PhysicsSim::AddTalonFX(TalonFX &talon, double const accelToFullTime, double const fullVel, bool const sensorPhase) {
    TalonFXSimProfile* simTalon = new TalonFXSimProfile(talon, accelToFullTime, fullVel, sensorPhase);
    this->_simProfiles.insert(this->_simProfiles.end(), simTalon);
}

void PhysicsSim::Run() {
    for (auto simProfile : _simProfiles) {
        simProfile->Run();
    }
}

double PhysicsSim::random(double min, double max) {
    return (max - min) / 2 * sin(fmod(rand(), 2 * 3.14159)) + (max + min) / 2;
}

double PhysicsSim::random(double max) {
    return random(0, max);
}
