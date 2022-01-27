#include <Shooter.h>
#include <AutoAimer.h>

Shooter::Shooter() {
    //TOOD: configure motors
    flywheel_slave->Follow(*flywheel_master, motorcontrol::FollowerType::FollowerType_AuxOutput1);
}

void Shooter::Periodic(double goal_rot) {
    switch(state) {
        case State::Zeroing:
            Zero(); 
            break;
        case State::Idle:
            flywheel_master->Set(ControlMode::PercentOutput, 0.0);
            break;
        case State::AutoAiming:
            AutoAim();
            break;
        case State::ManualAiming:
            ManualAim(goal_rot);
            break;
        case State::Shooting:
            Shoot();
            break;
    }
}

void Shooter::ManualAim(double goal_rot) {
    double turret_rot = turret->GetSelectedSensorPosition();
    
    if (abs(goal_rot) < 0.05) turret_rot = 0;
    if (goal_rot > 0 && turret_rot >= ShooterConstants::UpperRotLimit) turret_rot = 0;
    if (goal_rot < 0 && turret_rot <= ShooterConstants::LowerRotLimit) turret_rot = 0;

    turret->Set(ControlMode::PercentOutput, goal_rot*0.12); //not sure if this should be a constant?
}

//right now turret aims, then distance is read for flywheel & hood after - this can be changed
AutoAimer::Settings settings;
void Shooter::AutoAim() {
    double x_off = limelight.GetXOff(); 
    turret->Set(turretPID.Calculate(x_off, 0));
    if (turretPID.GetPositionError() > ShooterConstants::TurretTolerance) return;

    settings = auto_aimer.DistanceToSettings(limelight.CalculateDistance()); //TODO: replace 0 with sensor dist reading
    flywheel_master->Set(ControlMode::Velocity, settings.flywheel_speed);
    hood->Set(ControlMode::MotionMagic, settings.hood_pose);
}

bool Shooter::ReadyToShoot() {
    return (abs(settings.flywheel_speed - flywheel_master->GetSelectedSensorVelocity()) < ShooterConstants::FlywheelTolerance
        && abs(settings.hood_pose - hood->GetSelectedSensorPosition()) < ShooterConstants::HoodTolerance);
}

void Shooter::Shoot() { 
    flywheel_master->Set(ControlMode::Velocity, settings.flywheel_speed);
    hood->Set(ControlMode::MotionMagic, settings.hood_pose);
    if (!ReadyToShoot()) return;
    kicker->Set(ControlMode::Velocity, settings.kicker_speed);
}

void Shooter::Zero() {
    while (turret_limit_switch->Get()) {
        turret->Set(ControlMode::PercentOutput, 0.15);
    }
    turret->Set(ControlMode::PercentOutput, 0);
    turret->SetSelectedSensorPosition(0);
}