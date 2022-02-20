#include "Autonomous.h"
#include "Constants.h"
#include "AHRS.h"
#include "Trajectory.h"

AutoMode::AutoMode(Shooter& s, Intake& i, AHRS& a, SwerveDrive& sd) : shooter(s), intake(i), navx(a), swerve(sd) {
   // ResetAuto();
}

AutoMode::~AutoMode() {}


void AutoMode::ResetAuto() {
    swerve.ResetOdometry();
    shooter.setState(Shooter::State::IDLE);
    intake.setState(Intake::State::IDLE);
}

ThreeBallAuto::ThreeBallAuto(Shooter& s, Intake& i, AHRS& a, SwerveDrive& sd) : AutoMode(s, i, a, sd) {
    ResetAuto();
    actions = {IDLE, DRIVE, INTAKE, DRIVE, SHOOT, DRIVE, INTAKE, DRIVE, SHOOT};
    times = {15, 15, 15, 15, 15, 15, 15, 15, 15}; //todo: set
    trajpts = {Trajectory::Waypoint{-2, -37.4, 0, 180}, Trajectory::Waypoint{-304.65, -19.76, 0, 287.7}, 
                Trajectory::Waypoint{-190, -50, 0, 287.7}};
}


//lets have robot call periodic subsystem & swerve functions. auto class will update serve traj pts
void ThreeBallAuto::Periodic(double time) {
    if (EmergencyStop(time) || emergencyStopped) {
        emergencyStopped = true; return;
    } 
    if (isDone) return;

    switch(state) { //call correct action function. 
        case SHOOT:
            setState(Shoot(time));
            break;
        case INTAKE:
            setState(Intaking(time));
            break;
        case DRIVE:
            setState(Drive(time));
            break;
        case IDLE:
            setState(Idle(time));
            break;
    }
}

AutoMode::State ThreeBallAuto::Shoot(double time) {
    if (shooter.Aimed()) {
        if (timeWhenStartedShoot == -1) timeWhenStartedShoot = time;
        shooter.setState(Shooter::State::SHOOT); 
    }
    else shooter.setState(Shooter::State::AIM);

    if ((timeWhenStartedShoot != -1 && time >= timeWhenStartedShoot + timeToShoot) || time >= times[index]) { //condition to move on to next state
        index++;
        timeWhenStartedShoot = -1; //reset this
        if (index > actions.size()) isDone = true;
    }
    return actions[index];
}

AutoMode::State ThreeBallAuto::Intaking(double time) {
    intake.setState(Intake::State::RUN);
    shooter.setState(Shooter::State::LOAD);
    if (time >= times[index]) {
        index++;
    }
    return actions[index];
}

AutoMode::State ThreeBallAuto::Drive(double time) {
    if (stateJustChanged()) {
        swerve.ClearTraj();
        swerve.AddTrajPt(trajpts[index]); //may have to change if we want like a curved path (could have 2d array)
    }
    swerve.TrajectoryFollow(navx.GetYaw(), false);

//   if (swerve.AtTrajPoint(navx.GetYaw()) || time > times[index]) index++; //similarly may have to change
    return actions[index];

}

AutoMode::State ThreeBallAuto::Idle(double time) {
    return IDLE;
}

void ThreeBallAuto::setState(State newState){
    prevState = state;
    state = newState;
}

bool ThreeBallAuto::stateJustChanged() {
    return prevState != state;
}

bool ThreeBallAuto::EmergencyStop(double time) {return false;}




/* logic for blue, can mirror for red?:
    lets have the point be the middle of the robot
    robot is 36 inches both sides
    positive y is intake dir, pos x is right when intake facing away
    lets start that way

    first ball traj point:
        * rot = 180
        * x = -2
        * y = -37.4

    second ball traj point:
        * x = -304.65
        * y = -19.76 (subtracted (added) 16 to account for length of robot)
        * rot = 287.7
    
    third traj point to be closer to goal:
        * x = -190
        * y = -50
        * rot can stay same i think? 
    

- numbers are in inches, 0,0 is middle of hub
- start with bottom right corner (-110.448, 42.952)
- go to ball at (-126.434, 80.361), should be facing right way
- intake
- turn around 180 deg 
- shoot two balls
- turn like ~90 deg (todo: determine), go to ball at (-283.2, -118.472)
- intake
- turn around 180 deg, drive forward as needed
- shoot
*/
