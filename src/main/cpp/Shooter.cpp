/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Controls the shooter (including the channel)
  Incoporates limelight & channel

  Sets flywheel, turret, and hood motors to proper positions
  All shooter values are included

  Example: https://www.youtube.com/watch?v=YedaBdOery8&ab_channel=SheehyunKim
*/

#include <Shooter.h>
#include <iostream>

//sets motor configurations
//initializes data hash map values (input is limelight y offset, output is flywheel angle and speed)
Shooter::Shooter(){
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

    m_dataMap[-20.0] = {-1800, 12000};
    m_dataMap[-18.6] = {-1700, 11500};
    m_dataMap[-17.3] = {-1690, 11200};
    m_dataMap[-16.0] = {-1600, 10800};
    m_dataMap[-14.5] = {-1500, 10500};
    m_dataMap[-13.5] = {-1200, 9800};
    m_dataMap[-11.0] = {-900, 9500};
    m_dataMap[-9.0] = {-600, 9200};
    m_dataMap[-7.0] = {-450, 9000};
    m_dataMap[-5.0] = {-300, 8800};
    m_dataMap[-3.5] = {-200, 8600};
    m_dataMap[-1.0] = {-100, 8400};
    m_dataMap[0.0] = {-100, 8300};
    m_dataMap[3.32] = {0, 8000};
    m_dataMap[4.15] = {0, 7750};
    m_dataMap[6.99] = {0, 7750};
    m_dataMap[10.0] = {0, 7750};
    m_dataMap[20.0] = {0, 7750};

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
Shooter::Periodic(bool autonomous, double navX){

    m_autonomous = autonomous;
    //if target has gone past turret limits, set state to swivel so it'll swivel around to see it again
    if (m_swivelLeft || m_swivelRight) {
        setState(State::SWIVEL);
    }

    // Check the Color Values from REV Color Sensor
    // frc::SmartDashboard::PutNumber("Red", m_colorSensor.GetColor().red);
    // frc::SmartDashboard::PutNumber("blue", m_colorSensor.GetColor().blue);
    // frc::SmartDashboard::PutNumber("green", m_colorSensor.GetColor().green);

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
        case State::CLIMB:
            
            break;
        case State::LOAD:
            Load();
            break;
        case State::MANUAL:
            break;
        case State::BadIdea: //love the maturity (this was for outtake)
            // m_channel.setState(Channel::State::Badidea);
            break;
        case State::Tarmac:
            EdgeofTarmac();
            // m_channel.setState(Channel::State::RUN);
            break;
        case State::Hood:
            zeroHood();
            break;
        default:
            break;
    }
    // m_channel.Periodic();
    
    if (m_autonomous){
        return;
    }

    //the rest of this is for continuous aiming

    turretPosition = m_turret.GetSelectedSensorPosition();
    limelightXOff = m_limelight->getXOff();
    limelightYOff = m_limelight->getYOff();

    frc::SmartDashboard::PutNumber("hood position", m_hood.GetSelectedSensorPosition());

    // if we're swivelling, keep swivelling, and don't do anyting else.
    if (m_state == State::SWIVEL) {
       return;
    }    
    // aim at the target 
    double x_off = limelightXOff+2.0+thetaOff;
    double output = -m_turretController.Calculate(x_off);
    // limit motor output
    output = output > 0  && output > 0.38 ? 0.38: output;
    output = output < 0 && output < -0.38 ? -0.38: output;

     // if we don't see a target, stop spinning turret.
    if(limelightYOff == 0.000){
        m_turret.Set(ControlMode::PercentOutput, 0.0);
        return;
    }   

    MovementOffsetCalculation(getFieldAng(navX)); //for shooting while moving
    
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
    // frc::SmartDashboard::PutBoolean("leftSwivel", m_swivelLeft);
    // frc::SmartDashboard::PutBoolean("rightSwivel", m_swivelRight);
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

    //Check if the limelight sees the target  
    double point = m_limelight->getYOff();
    // Comment this out if we're getting new shooter points

    // dataMap is the calibration for elevation for the hood and flywheel (the hood position & flywheel speed for limelight angle).
    // auto data = m_dataMap.find(point);
    double point1, point2;
    if(withinRange(m_dataPoints, point, point1, point2)){
        auto data1 = m_dataMap[point1];
            auto data2 = m_dataMap[point2];
            double angle1, speed1;
            double angle2, speed2;
            angle1 = data1.first;
            speed1 = data1.second;
            angle2 = data2.first;
            speed2 = data2.second;

            //linear piecewise interpolation
            double interval = point1 - point2;
            double xdiff = point - point1;
            m_angle = (((angle2 - angle1)/interval * xdiff) + angle1) * m_angle_scale_factor;
            m_speed = (((speed2 - speed1)/interval * xdiff) + speed1) * m_speed_scale_factor;

            // average
            // m_angle = (angle1 + angle2)/2 * angle_scale_factor;
            // m_speed = (speed1 + speed2)/2 * speed_scale_factor;

        }else{
            m_angle = 0;
            m_speed = 0;
        }
    
    
    // frc::SmartDashboard::PutNumber("turret position", m_turret.GetSelectedSensorPosition());

    // Set Turret movement for auto mode
    if(m_autonomous){
        // std::cout << "does it get here?" << std::endl;
        
        double x_off = limelightXOff+2.0;
        double output = -m_turretController.Calculate(x_off);
        output = output > 0  && output > 0.38 ? 0.38: output;
        output = output < 0 && output < -0.38 ? -0.38: output;
        frc::SmartDashboard::PutNumber("output", output);
        // // if we don't see the target, this will stop us from shooting blindly.
        // // hopefully we see the target next cycle
        // if(turretPosition > ShooterConstants::turretMax &&
        //     output > 0){
        //     m_turret.Set(ControlMode::PercentOutput, 0.0);  
        //     return;
        // } else if(turretPosition < ShooterConstants::turretMin &&
        //     output < 0){
        //     m_turret.Set(ControlMode::PercentOutput, 0.0);
        //     return;
        // } else {
        m_turret.Set(output);
        // }
    }

    // Can see the target, set flywheel and hood appropriately
    // Set Flywheel Velocity
    m_flywheelMaster.Set(ControlMode::Velocity, m_speed);
    m_flywheelSlave.Set(ControlMode::Velocity, -m_speed);
    // frc::SmartDashboard::PutNumber("flywheel speed", m_flywheelMaster.GetSelectedSensorVelocity());
    // frc::SmartDashboard::PutNumber("hood angle", m_hood.GetSelectedSensorPosition());
    
    // Set Hood Position
    if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
        m_hood.Set(ControlMode::PercentOutput, 0.0);
    } else {
        m_hood.Set(ControlMode::Position, m_angle);
    }
    
}

//sets turret to position that allows robot to climb
void
Shooter::Climb(){
    m_turret.Set(ControlMode::Position, -30000);
}


// Deprecated function
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

    // frc::SmartDashboard::PutBoolean("wheel", flywheelReady);
    // frc::SmartDashboard::PutBoolean("hood", hoodReady);
    // frc::SmartDashboard::PutBoolean("turret", turretReady);

    if(flywheelReady && hoodReady ){
        Load();
    }
}


// Binary search for shooter point values
// Return true if the point is within out data Map
//since p1 and p2 are references, the doubles passed in will be set to the correct values 
bool
Shooter::withinRange(std::vector<double>& array, double p, double& p1, double& p2){
    int left =0;
    int right = array.size() -1;
    if(p < array[left] || p > array[right]){
        return false;
    }
    int mid = (left + right)/2;
    while(left < right){
        if(array[mid] <= p && p <= array[mid+1]){
            p1 = array[mid];
            p2 = array[mid+1];
            return true;
        } else if(array[mid] <= p){
            left = mid;
        } else if(array[mid] >= p){
            right = mid;
        }
        mid = (left + right)/2;
    }
    return false;
}


//Shoot Function 
void
Shooter::Shoot(){
    if(Aimed()){
        Load(); //run the channel appropriate for ball color & state
    }
}


// Zero Function
// This will reset all of your motor positions. 
void
Shooter::Zero(){
    m_turret.SetSelectedSensorPosition(-31500);
    m_turretZero = true;
    m_hood.SetSelectedSensorPosition(0);
}


// Zero the hood
void
Shooter::zeroHood(){
    // if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
    //     m_hood.Set(ControlMode::PercentOutput, 0.0);
    //     m_hood.SetSelectedSensorPosition(0); 
    //     return;
    // } else {
    //     m_hood.Set(ControlMode::PercentOutput, 0.30);
    // }
    m_hood.SetSelectedSensorPosition(0);
    // if(!m_hoodZero){
    // m_hood.Set(ControlMode::PercentOutput, -0.30);   
    // }
}


//Load Function
void
Shooter::Load(){

    // m_kicker.Set(ControlMode::PercentOutput, 0.25);
    // Shooter v1 was 0.2 percent output
    if(m_state == State::SHOOT || m_state == State::Tarmac){
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
    bool hoodReady = abs(m_hood.GetSelectedSensorPosition() - m_angle) < 150; // make this interval smaller
    bool turretReady = abs(limelightXOff+2.0+thetaOff) < 2.5;
    frc::SmartDashboard::PutBoolean("flywheelReady", flywheelReady);
    frc::SmartDashboard::PutBoolean("hoodReady", hoodReady);
    frc::SmartDashboard::PutBoolean("turretReady", turretReady);

    //Add turret ready
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


//Convert unit per 100 ms to rpm 
double
Shooter::convertToRPM(double ticks){
    return ticks/ShooterConstants::TalonFXunitsperrot*600;
}


// Deprecated function
void
Shooter::DisableMotors(){
    m_kicker.Disable();
    m_hood.Disable();
    m_flywheelMaster.Disable();
    m_flywheelSlave.Disable();
    // m_channel.DisableMotor();
}


// Set the ball color ejection
void
Shooter::setColor(bool isblue){
    m_blue = isblue;
    m_ballColor = m_blue? m_redBall : m_blueBall;
}


// Low goal hardpoint
void
Shooter::LowShot(){
    m_hood.Set(ControlMode::Position, 1500);
    m_flywheelMaster.Set(ControlMode::PercentOutput, 0.5);
    m_flywheelSlave.Set(ControlMode::PercentOutput, -0.5);
}


// Unused function originally for moving turret to field oriented positions
void
Shooter::peekTurret(double navX, double POV){
    const double robotAngRatio = 63000.0/360.0;
    // const double robotAngRatio = -63000/360.0;
    double POV_deg = POV>180? POV-360:POV;
    double navX_deg = -navX;
    double target_turret_tick = (POV-navX_deg);

    // convert above scale to -63000 to 0
    target_turret_tick = target_turret_tick<0? target_turret_tick:target_turret_tick-360;
    target_turret_tick*=robotAngRatio;
    // frc::SmartDashboard::PutNumber("target value", target_turret_tick);
    
    // double intervalTarget;

    // if(target_turret_tick<ShooterConstants::turretMax && target_turret_tick>ShooterConstants::turretMin){
    //     double intervalTarget = target_turret_tick-m_turret.GetSelectedSensorPosition();
    //     intervalTarget = intervalTarget > turnInterval? turnInterval: (intervalTarget < -turnInterval? -turnInterval: intervalTarget);
    // }

    m_turret.Set(ControlMode::Position, target_turret_tick);
}


// Turn on limelight LEDs
void
Shooter::enablelimelight(){
    m_limelight->setLEDMode("ON");
}


//sets odometry to passed in odometry object
void
Shooter::GetOdom(Odometry *odom){
    m_odom = odom;
}


// Shooting while moving math
// This should be called whenever we start to shoot when the robot is in motion
void
Shooter::MovementOffsetCalculation(double angle){
    
    m_goalOffset.first = -m_odom->getXSpeed();
    m_goalOffset.second = -m_odom->getYSpeed();
 
    double distance = m_limelight->calculateDistance();

    // ~ 1.0 when distance ~4
    // ~ 0.8 when distance ~3
    // ~ 0.6 when distance ~2

    double multiplier = 0.85;

    double offMagnitude = multiplier*sqrt((m_goalOffset.first * m_goalOffset.first) + 
        (m_goalOffset.second * m_goalOffset.second));
    double a = 0 - angle - turnTheta;

    double sus = 0; //high quality variable name

    // Check for robot movement
    if(abs(m_goalOffset.first) > std::numeric_limits<double>::epsilon()
        || abs(m_goalOffset.second) > std::numeric_limits<double>::epsilon()){
        sus = atan2(m_goalOffset.second, m_goalOffset.first);
    } 

    double theta1 = (3.1415/2 - sus) + a*3.1415/180;

    // The new distance to set the shooter to
    dOff = sqrt(distance*distance + offMagnitude*offMagnitude -
        2*distance*offMagnitude*cos(theta1));
    double dDelta = abs(distance - dOff);

    dOff = (dOff > distance)? dOff-2*dDelta: dOff + 2*dDelta;
    dOff = (dOff == 0)? distance: dOff;

    dOffConverted = (atan(2.1/dOff)*180/3.1415)-40;

    double asinInterval = offMagnitude * sin(theta1) / dOff;
    asinInterval = std::clamp(asinInterval, -1.0, 1.0);

    thetaOff = asin(asinInterval)*180/3.1415;
    turnTheta = thetaOff;

    thetaOff = std::clamp(thetaOff, -ShooterConstants::maxOffset, ShooterConstants::maxOffset);

    // frc::SmartDashboard::PutNumber("X", m_goalOffset.first);
    // frc::SmartDashboard::PutNumber("Y", m_goalOffset.second);
    // frc::SmartDashboard::PutNumber("distance", distance);
    // frc::SmartDashboard::PutNumber("theta1", theta1);
    // frc::SmartDashboard::PutNumber("DOFF", dOff);
}


// Finds the field relative angle of the turret
double
Shooter::getFieldAng(double navX){
    double navX_deg = -navX;
    double turretAng = turretPosition/175;
    double ang = turretAng+navX_deg+360.0;
    return ang<0?ang+360:(ang>360?ang-360:ang);
}