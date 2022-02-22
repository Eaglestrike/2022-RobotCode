// #include "Shooter.h"
// #include "Intake.h"
// #include "SwerveDrive.h"
// #include <AHRS.h>
// #include <frc/TimedRobot.h>

//  class AutoMode {  
//     public:
//         AutoMode(Shooter& s, Intake& i, AHRS& a, SwerveDrive& sd);
//         virtual ~AutoMode();

//         enum State{
//             SHOOT,
//             INTAKE,
//             DRIVE,
//             IDLE,
//         };

//         std::vector<State> actions;  
//         std::vector<double> times; //the times are when you HAVE to move on. if state is completed it can move on early, but this is hard limit
//         std::vector<Trajectory::Waypoint> trajpts;

//         void ResetAuto();
//         virtual void Periodic(double time) {}
//         State getState() {return state;}

//     protected:
//         State state;

//         Shooter& shooter;
//         Intake& intake;
//         AHRS& navx;
//         SwerveDrive& swerve;
// };         


// class DriveAuto : AutoMode {
//     //Andrew: put your drive auto here
// };

// class ThreeBallAuto : AutoMode {
//     public:
//         ThreeBallAuto(Shooter& s, Intake& i, AHRS& a, SwerveDrive& sd);

//         void Periodic(double time) override;

//         State Shoot(double time);
//         State Intaking(double time);
//         State Drive(double time);
//         State Idle(double time);
//         bool EmergencyStop(double time);

//         State getState() {return state;}
//         void setState(State newState);
//         bool stateJustChanged();

//     private:
//         State state = IDLE;
//         State prevState = IDLE;

//         double timeWhenStartedShoot = -1;
//         double timeToShoot = 3; //will be different from 2 to 1 balls, but shooting 1 ball is last thing we do so ok if over time

//         int index;

//         bool emergencyStopped = false;
//         bool isDone = false;

// };