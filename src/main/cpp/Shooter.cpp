#include <Shooter.h>
#include <iostream>

//sets motor configurations
//initializes data hash map values (input is limelight y offset, output is flywheel angle and speed)
Shooter::Shooter(Swerve& s, Limelight* l) : swerve(s) {
    m_limelight = l;

    m_flywheelMaster.SetNeutralMode(NeutralMode::Coast);
    m_flywheelSlave.SetNeutralMode(NeutralMode::Coast);
    m_hood.SetNeutralMode(NeutralMode::Brake);
    m_kicker.SetNeutralMode(NeutralMode::Brake);
    m_turret.SetNeutralMode(NeutralMode::Brake);

    m_flywheelMaster.SetSafetyEnabled(false);
    m_flywheelSlave.SetSafetyEnabled(false);
    m_turret.SetSafetyEnabled(false);
    
    m_swivelRight = false;
    m_swivelLeft = false;
    
    m_flywheelMaster.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    m_flywheelSlave.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    m_turret.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    m_kicker.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 100);

    m_flywheelMaster.Config_kF(0, ShooterConstants::flywheelF);
    m_flywheelMaster.Config_kP(0, ShooterConstants::flywheelP);
    m_flywheelMaster.Config_kI(0, ShooterConstants::flywheelI);
    m_flywheelMaster.Config_kD(0, ShooterConstants::flywheelD);

    m_flywheelSlave.Config_kF(0, ShooterConstants::flywheelF);
    m_flywheelSlave.Config_kP(0, ShooterConstants::flywheelP);
    m_flywheelSlave.Config_kI(0, ShooterConstants::flywheelI);
    m_flywheelSlave.Config_kD(0, ShooterConstants::flywheelD);

    m_hood.Config_kP(0, ShooterConstants::hoodP);
    m_hood.Config_kI(0, ShooterConstants::hoodI);
    m_hood.Config_kD(0, ShooterConstants::hoodD);
    m_hood.Config_kF(0, ShooterConstants::hoodF);

    m_turret.Config_kP(0, ShooterConstants::turretPosP);
    m_turret.Config_kI(0, ShooterConstants::turretPosI);
    m_turret.Config_kD(0, ShooterConstants::turretPosD);

    m_hoodZero = false;
    m_turretZero = false;

    m_colorMatcher.AddColorMatch(m_redBall);
    m_colorMatcher.AddColorMatch(m_blueBall);
    m_colorMatcher.AddColorMatch(m_defualtColor);
}


// Deconstructor
Shooter::~Shooter(){
    // delete m_limelight;
}


//Periodic Function is called every 20 milliseconds
//call appropriate action functions based on state
void
Shooter::Periodic(bool autonomous){

    m_autonomous = autonomous;
    //if target has gone past turret limits, set state to swivel so it'll swivel around to see it again
    if (m_swivelLeft || m_swivelRight) {
        setState(State::SWIVEL);
    }

    // Some states are "dead" ~ no longer in use
    switch(m_state){
        case State::SWIVEL:
            Swivel();
            break;
        case State::SHOOT:
            Aim();
            Shoot();
            // m_channel.setState(Channel::State::RUN);
            break;
        case State::IDLE:
            Stop();
            break;
        case State::AIM:
            Aim();
            break;
        case State::LOAD:
            Load();
            break;
        case State::MANUAL:
            break;
       case State::HOOD:
            zeroHood();
            break;
        default:
            break;
    }
    // m_channel.Periodic();
    
    if (m_autonomous){
        return;
    }

    //this replaces the binary search thing for getting flywheel speed & hood angle
    //it is my belief that due to the small number of entries the binary seach will actually be slower, but we can do benchmarking if someone disagrees
    ShooterCalc::Settings settings = calc.calculate();

    TurretAim(settings.xoff_offset);
    
}

void Shooter::TurretAim(double offset) {
    turretPosition = m_turret.GetSelectedSensorPosition();
    limelightXOff = m_limelight->getXOff();
    limelightYOff = m_limelight->getYOff();

    frc::SmartDashboard::PutNumber("hood position", m_hood.GetSelectedSensorPosition());

    if (m_state == State::SWIVEL) {
       return;
    }    
    // aim at the target 
    double x_off = limelightXOff+2.0;
    double output = -m_turretController.Calculate(x_off + offset);
    
    // limit motor output
    output = std::clamp(output, -0.38, 0.28);

     // if we don't see a target, stop spinning turret.
    if(limelightYOff == 0.000){
        m_turret.Set(ControlMode::PercentOutput, 0.0);
        return;
    }   
    
    // if we're trying to rotate past the limit, swing around and start SWIVEL state.
    if(turretPosition > ShooterConstants::turretMax &&
        output > 0){
        m_swivelRight = true;
        m_swivelLeft = false;
        setState(State::SWIVEL);
    }
    else if(turretPosition < ShooterConstants::turretMin &&
        output < 0){
        m_swivelLeft = true;
        m_swivelRight = false;
        setState(State::SWIVEL);
    }
    else {
        // target is in sight, keep aiming at it.
        m_turret.Set(output);
    }

}


//assumes target is at other limit (so like 0 -> 360 type deal), goes to that
//idk how he came up with the numbers but it seems to work so I'd trust them
void Shooter::Swivel(){
    if(m_swivelRight){
        m_turret.Set(ControlMode::Position, -60000);
            // should stop swivelling when within 2000 units of target, which is -60,000                
            if((abs(turretPosition + 60000) < 2000) ||
                (turretPosition < -30000 && limelightXOff != 0.000)){
                m_swivelRight = false;
                setState(State::IDLE);
            }
    } else if(m_swivelLeft){
        m_turret.Set(ControlMode::Position, -3000);
            // should stop swivelling when within 2000 units of target, which is -3,000
            if((abs(turretPosition + 3000) < 2000)||
                (turretPosition > -30000 && limelightXOff != 0.000)){
                m_swivelLeft = false;
                setState(State::IDLE);
            }
    }
    // shouldn't get here, so doing nothing.
}


// Ball Ejection when it is the wrong color
//will get shot slowly out shooter
void Shooter::EjectBall() {
        m_speed = 3000;
        m_angle = -100;
        m_hood.Set(ControlMode::Position, m_angle);
        m_flywheelMaster.Set(ControlMode::Velocity, m_speed);
        m_flywheelSlave.Set(ControlMode::Velocity, -m_speed);
        // m_channel.Run();        
        Load();
}


//Aim Function for Turret
void
Shooter::Aim(){

    //  If the ball is the wrong color, eject it...
    if(m_colorMatcher.MatchClosestColor(m_colorSensor.GetColor(), m_confidence) == m_ballColor){
      EjectBall();
      return;
    }

    //in the past it's been suspicious if the TA limelight value is actually accurate
    //TOOD: testing, could check for tx/ty being default value instead
    if (!m_limelight->targetAquired() || m_limelight->getXOff() > 1000) return;
 
    if (settings.hood_angle == NAN) return;
    m_angle = settings.hood_angle;
    m_speed = settings.flywheel_speed;

    // Set Turret movement for auto mode
    //I'm afraid to get rid of this because it may be important, but there should be no reason to have this
    //and it may be problematic if the auto turret aim from periodic is conflicting with this
    if(m_autonomous){
        
        double x_off = m_limelight->getXOff() + 2.0 + settings.xoff_offset;
        double output = -m_turretController.Calculate(x_off);
        output = output > 0  && output > 0.38 ? 0.38: output;
        output = output < 0 && output < -0.38 ? -0.38: output;
        m_turret.Set(output);
    }

    //Set flywheel velocity
    //I'd like to pair these instead of setting them indivdually, but since they only get set once that isn't a priority
    m_flywheelMaster.Set(ControlMode::Velocity, m_speed);
    m_flywheelSlave.Set(ControlMode::Velocity, -m_speed);
    
    // Set Hood Position
    if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){ //we are totally far down, don't keep going
        m_hood.Set(ControlMode::PercentOutput, 0.0);
    } else {
        m_hood.Set(ControlMode::Position, m_angle);
    }
    
}

//Shoot Function 
void
Shooter::Shoot(){
    if(Aimed()){
        Load(); //run the channel appropriate for ball color & state
    }
}

//sets turret to position that allows robot to climb
void
Shooter::Climb(){
    m_turret.Set(ControlMode::Position, -30000);
}


//Zero motor positions at start of game
void
Shooter::Zero(){
    m_turret.SetSelectedSensorPosition(-31500);
    m_turretZero = true;
    m_hood.SetSelectedSensorPosition(0);
}


// Zero the hood
//in the past there's been things of going down until we got a current
//but the problem prompting us to try and zero hood mid-game was largely removed so it's not there anymore
void
Shooter::zeroHood(){
    m_hood.SetSelectedSensorPosition(0);
}


//Load Function
void
Shooter::Load(){

    // m_kicker.Set(ControlMode::PercentOutput, 0.25);
    // Shooter v1 was 0.2 percent output
    if(m_state == State::SHOOT /*|| m_state == State::Tarmac*/){
        m_kicker.Set(ControlMode::PercentOutput, 0.25);
    } 
    else if(m_state == State::LOAD){ //intaking not shooting
        //if a ball gets far enough in the channel, stop to prevent it getting stuck in the flywheel
        if(!m_photogate.Get() && !m_photogate2.Get()){ 
            m_kicker.Set(ControlMode::PercentOutput, 0.0);
            // m_channel.setState(Channel::State::IDLE);
        } 
        else if(m_photogate2.Get() && !m_photogate.Get()){
            // m_channel.setState(Channel::State::RUN);
            m_kicker.Set(ControlMode::PercentOutput, 0.0);
        }
        //okay to keep moving balls through channel
        else if(!m_photogate2.Get() && m_photogate.Get()){
            m_kicker.Set(ControlMode::PercentOutput, 0.2);
            // m_channel.setState(Channel::State::RUN);
        }
        else {
            m_kicker.Set(ControlMode::PercentOutput, 0.2);
            // m_channel.setState(Channel::State::RUN);
        }
    }
}


//set the State of the shooter
void
Shooter::setState(State newState){
    m_state = newState;
}


//Check if the turret, flywheel, and hood is aimed (so we won't shoot if we aren't aimed)
bool
Shooter::Aimed(){
    bool flywheelReady = abs(m_flywheelMaster.GetSelectedSensorVelocity() - m_speed) < 300;
    bool hoodReady = abs(m_hood.GetSelectedSensorPosition() - m_angle) < 150; 
    bool turretReady = abs(limelightXOff+2.0) < 2.5;

    if( flywheelReady && hoodReady && turretReady){ 
        return true;
    } else {
        return false;
    }
}



//Calibrate function
// Helper function
// Use to tune the shooter
void
Shooter::Calibrate(){
    // std::cout << "gets here" << std::endl;
    m_flywheelMaster.Set(ControlMode::Velocity, m_speed);
    m_flywheelSlave.Set(ControlMode::Velocity, -m_speed);

    m_hood.Set(ControlMode::Position, m_angle);

    Shoot();

    // double x_off = limelightXOff+2.0;
    // double output = -m_turretController.Calculate(x_off);
    // frc::SmartDashboard::PutNumber("output", output);
    // // output = output > 0  && output > 0.38 ? 0.38: output;
    // // output = output < 0 && output < -0.38 ? -0.38: output;
    // m_turret.Set(output);
}


//Helper function to tune shooter PID
void
Shooter::setPID(){
    // double F = frc::SmartDashboard::GetNumber("F", 0.0);
    // frc::SmartDashboard::PutNumber("F", F);
    // double P = frc::SmartDashboard::GetNumber("P", 0.0);
    // frc::SmartDashboard::PutNumber("P", P);
    // double I = frc::SmartDashboard::GetNumber("I", 0.0);
    // frc::SmartDashboard::PutNumber("I", I);
    // double D = frc::SmartDashboard::GetNumber("D", 0.0);
    // frc::SmartDashboard::PutNumber("D", D);

    m_speed = frc::SmartDashboard::GetNumber("speed", 0.0);
    m_angle = frc::SmartDashboard::GetNumber("angle", 0.0);
    frc::SmartDashboard::PutNumber("speed", m_speed);
    frc::SmartDashboard::PutNumber("angle", m_angle);

    // Unused controller
    // m_turretPos.SetPID(P, I, D);
    // m_turret.Config_kP(0, P);
    // m_turret.Config_kI(0, I);
    // m_turret.Config_kD(0, D);

    // m_flywheelMaster.Config_kF(0, F);
    // m_flywheelMaster.Config_kP(0, P);
    // m_flywheelMaster.Config_kI(0, I);
    // m_flywheelMaster.Config_kD(0, D);

    // m_flywheelSlave.Config_kF(0, F);
    // m_flywheelSlave.Config_kP(0, P);
    // m_flywheelSlave.Config_kI(0, I);
    // m_flywheelSlave.Config_kD(0, D);
    
    // m_turretController.SetPID(P, I, D);
    
    // m_hood.Config_kP(0, P);
    // m_hood.Config_kI(0, I);
    // m_hood.Config_kD(0, D);
    // m_hood.Config_kF(0, F);
    
}


//Stop All shooter movements
void
Shooter::Stop(){
    // m_turret.Set(0);
    m_flywheelSlave.Set(0);
    m_flywheelMaster.Set(0);
    m_kicker.Set(0);
    m_hood.Set(0);
    // m_channel.setState(Channel::State::IDLE);
}


//Manual turret movement
void
Shooter::Manual(double turret_rot){
    if(abs(turret_rot) <= 0.05){
        turret_rot = 0;
    }
    if(turret_rot > 0 && m_turret.GetSelectedSensorPosition() > ShooterConstants::turretMax){
        turret_rot = 0;
    }
    if(turret_rot < 0 && m_turret.GetSelectedSensorPosition() < ShooterConstants::turretMin){
        turret_rot = 0;
    }
    //frc::SmartDashboard::PutNumber("manual", turret_rot * 0.2);
    m_turret.Set(ControlMode::PercentOutput, turret_rot*0.2);
}


// Set the ball color ejection
void
Shooter::setColor(bool isblue){
    m_blue = isblue;
    m_ballColor = m_blue? m_redBall : m_blueBall;
}


// Hard coded to shoot from specified distance
void
Shooter::EdgeofTarmac(){
    // Intended to dump the balls into the low bucket.

    m_flywheelMaster.Set(ControlMode::Velocity, m_tarmac_speed);
    m_flywheelSlave.Set(ControlMode::Velocity, -m_tarmac_speed);

    m_hood.Set(ControlMode::Position, m_tarmac_angle);
    if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
        m_hood.Set(ControlMode::PercentOutput, 0.0);
    }

    bool flywheelReady = abs(m_flywheelMaster.GetSelectedSensorVelocity() - m_tarmac_speed) < 400;
    bool hoodReady = abs(m_hood.GetSelectedSensorPosition() - m_tarmac_angle) < 100; // make this interval smaller
    // bool turretReady = abs(m_limelight->getXOff()+0.0) < 0.5;

    if(flywheelReady && hoodReady ){
        Load();
    }
}

// Low goal hardpoint
void
Shooter::LowShot(){
    m_hood.Set(ControlMode::Position, 1500);
    m_flywheelMaster.Set(ControlMode::PercentOutput, 0.5);
    m_flywheelSlave.Set(ControlMode::PercentOutput, -0.5);
}



// Turn on limelight LEDs
void
Shooter::enablelimelight(){
    m_limelight->setLEDMode("ON");
}
