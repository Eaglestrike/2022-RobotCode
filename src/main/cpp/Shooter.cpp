#include <Shooter.h>
#include <iostream>


Shooter::Shooter(){
    m_flywheelMaster.SetNeutralMode(NeutralMode::Coast);
    m_flywheelSlave.SetNeutralMode(NeutralMode::Coast);
    m_hood.SetNeutralMode(NeutralMode::Coast);
    m_kicker.SetNeutralMode(NeutralMode::Brake);
    m_turret.SetNeutralMode(NeutralMode::Brake);

    m_flywheelMaster.SetSafetyEnabled(false);
    m_flywheelSlave.SetSafetyEnabled(false);
    m_turret.SetSafetyEnabled(false);
    // m_turret.
    
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

    // m_turretPos.EnableContinuousInput(-1000, -63000);

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

    dataMap[-19.4] = {5700, 17200};
    dataMap[-18.9] = {5700, 16400};
    dataMap[-17.5] = {5600, 15900};
    dataMap[-14.8] = {5600, 14900};
    dataMap[-12.5] = {5600, 14000};
    dataMap[-9.5] = {5500, 13500};
    dataMap[-7.4] = {5400, 13200};
    dataMap[-5.4] = {5200, 12600};
    dataMap[-1.8] = {4800, 12300};
    dataMap[0.5] = {4700, 12200};
    dataMap[1.7] = {4500, 12000};
    dataMap[3.65] = {4100, 12000};
    dataMap[6.0] = {3800, 11800};
    dataMap[9.9] = {2800, 11700};
    dataMap[14.2] = {2800, 11500};
    dataMap[16.0] = {2600, 11000};
    dataMap[19.88] = {1800, 10500};

    m_hoodZero = false;
    m_turretZero = false;

    m_colorMatcher.AddColorMatch(redBall);
    m_colorMatcher.AddColorMatch(blueBall);
    m_colorMatcher.AddColorMatch(defualtColor);
}


Shooter::~Shooter(){}


//Periodic Function
void
Shooter::Periodic(){

    switch(m_state){
        case State::SHOOT:
            Aim();
            Shoot();
            m_channel.setState(Channel::State::RUN);
            break;
        case State::IDLE:
            Stop();
            break;
        case State::AIM:
            Aim();
            break;
        case State::ZERO:
            Zero();
            break;
        case State::LOAD:
            Load();
            break;
        case State::PEEK:
            break;
        case State::MANUAL:
            break;
        case State::BadIdea:
            m_channel.setState(Channel::State::Badidea);
            break;
        case State::Tarmac:
            EdgeofTarmac();
            // Shoot();
            m_channel.setState(Channel::State::RUN);
        case State::Hood:
            zeroHood();
        default:
            break;
    }
    m_channel.Periodic();

    //frc::SmartDashboard::PutNumber("Red", m_colorSensor.GetColor().red);
    //frc::SmartDashboard::PutNumber("blue", m_colorSensor.GetColor().blue);
    //frc::SmartDashboard::PutNumber("green", m_colorSensor.GetColor().green);

}


//Aim Function for Turret
void
Shooter::Aim(){
    //m_limelight->setLEDMode("ON");

    if(m_colorMatcher.MatchClosestColor(m_colorSensor.GetColor(), confidence) == ballColor){
        m_speed = 8000;
        m_angle = 100;
        m_hood.Set(ControlMode::Position, m_angle);
        m_flywheelMaster.Set(ControlMode::Velocity, m_speed);
        m_flywheelSlave.Set(ControlMode::Velocity, -m_speed);
        m_channel.Run();
        
        Load();
        return;
    }

    //Check if the limelight sees the target
    double point = m_limelight->getYOff();
    frc::SmartDashboard::PutNumber("yoff", point);
    if(point == 0.000){
        m_channel.setState(Channel::State::IDLE);
        return;
    }
    

    // Comment this for shooter recalibration

    auto data = dataMap.find(point);
    if(data != dataMap.end()){
        m_angle = data->second.first;
        m_speed = data->second.second;
    } else {
        double point1, point2;
        if(withinRange(dataPoints, point, point1, point2)){
            auto data1 = dataMap[point1];
            auto data2 = dataMap[point2];
            double angle1, speed1;
            double angle2, speed2;
            angle1 = data1.first;
            speed1 = data1.second;
            angle2 = data2.first;
            speed2 = data2.second;

            // interpolate - 
            double interval = point1 - point2;
            double xdiff = point - point1;
            m_angle = (((angle2 - angle1)/interval * xdiff) + angle1) * angle_scale_factor;
            m_speed = (((speed2 - speed1)/interval * xdiff) + speed1) * speed_scale_factor;

            // taking the average - 
            // m_angle = (angle1 + angle2)/2 * angle_scale_factor;
            // m_speed = (speed1 + speed2)/2 * speed_scale_factor;

        }else{
            m_angle = 0;
            m_speed = 0;
        }
    }
    
    // frc::SmartDashboard::PutNumber("turret position", m_turret.GetSelectedSensorPosition());

    // Set Turret movement
    double x_off = m_limelight->getXOff()+4.3;
    double output = -m_turretController.Calculate(x_off);
    output = output > 0  && output > 0.38 ? 0.38: output;
    output = output < 0 && output < -0.38 ? -0.38: output;
    if(m_turret.GetSelectedSensorPosition() > ShooterConstants::turretMax &&
        output > 0){
        m_turret.Set(ControlMode::PercentOutput, 0.0);  
        return;
    }
    if(m_turret.GetSelectedSensorPosition() < ShooterConstants::turretMin &&
        output < 0){
        m_turret.Set(ControlMode::PercentOutput, 0.0);
        return;
    }
    else {
        m_turret.Set(output);
    }

    // Set Flywheel Velocity
    m_flywheelMaster.Set(ControlMode::Velocity, m_speed);
    m_flywheelSlave.Set(ControlMode::Velocity, -m_speed);
    frc::SmartDashboard::PutNumber("flywheel speed", m_flywheelMaster.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("hood angle", m_hood.GetSelectedSensorPosition());
    
    // Set Hood Position
    if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
         m_hood.Set(ControlMode::PercentOutput, 0.0);
         return;
    } 
    else if(m_hood.GetSelectedSensorPosition() > ShooterConstants::hoodMax &&
        m_angle >= ShooterConstants::hoodMax){
        m_hood.Set(ControlMode::PercentOutput, 0.0); 
        return;
    }
    else if(m_hood.GetSelectedSensorPosition() < ShooterConstants::hoodMin &&
        m_angle <= ShooterConstants::hoodMin){
        m_hood.Set(ControlMode::PercentOutput, 0.0);
        return;
    }
    else {
        m_hood.Set(ControlMode::Position, m_angle);
    }
    //frc::SmartDashboard::PutNumber("yOff", m_limelight->getYOff());
    //frc::SmartDashboard::PutNumber("xOff", m_limelight->getXOff());
}


void
Shooter::EdgeofTarmac(){
    // double x_off = m_limelight->getXOff()+4.3;
    // double output = -m_turretController.Calculate(x_off);
    // output = output > 0  && output > 0.38 ? 0.38: output;
    // output = output < 0 && output < -0.38 ? -0.38: output;
    // if(m_turret.GetSelectedSensorPosition() > ShooterConstants::turretMax &&
    //     output > 0){
    //     m_turret.Set(ControlMode::PercentOutput, 0.0);  
    //     return;
    // }
    // if(m_turret.GetSelectedSensorPosition() < ShooterConstants::turretMin &&
    //     output < 0){
    //     m_turret.Set(ControlMode::PercentOutput, 0.0);
    //     return;
    // }
    // else {
    //     m_turret.Set(output);
    // }

    m_flywheelMaster.Set(ControlMode::Velocity, m_tarmac_speed);
    m_flywheelSlave.Set(ControlMode::Velocity, -m_tarmac_speed);

    m_hood.Set(ControlMode::Position, m_tarmac_angle);
    if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
        m_hood.Set(ControlMode::PercentOutput, 0.0);
    }

    bool flywheelReady = abs(m_flywheelMaster.GetSelectedSensorVelocity() - m_tarmac_speed) < 400;
    bool hoodReady = abs(m_hood.GetSelectedSensorPosition() - m_tarmac_angle) < 100; // make this interval smaller
    // bool turretReady = abs(m_limelight->getXOff()+4.3) < 2.5;

    frc::SmartDashboard::PutBoolean("wheel", flywheelReady);
    frc::SmartDashboard::PutBoolean("hood", hoodReady);
    // frc::SmartDashboard::PutBoolean("turret", turretReady);

    if(flywheelReady && hoodReady ){
        Load();
    }
}


//Interpolate shooter point values
bool
Shooter::withinRange(std::vector<double>& array, double p, double& p1, double& p2){
    int left =0;
    int right = array.size() -1;
    if(p < array[left] || p > array[right]){
        // std::cout << "bad1" <<std::endl;
        return false;
    }
    int mid = (left + right)/2;
    while(left < right){
        if(array[mid] <= p && p <= array[mid+1]){
            p1 = array[mid];
            p2 = array[mid+1];
            return true;
            // std::cout << "does find point" <<std::endl;
        } else if(array[mid] <= p){
            left = mid;
        } else if(array[mid] >= p){
            right = mid;
        }
        mid = (left + right)/2;
    }
    // std::cout << "bad2" <<std::endl;
    return false;
}


//Shoot Function 
void
Shooter::Shoot(){
    if(Aimed()){
        Load();
    }
}


//Zero Function
//fix
void
Shooter::Zero(){
    m_turret.SetSelectedSensorPosition(-31500);
    m_turretZero = true;
    //NOTE: REPLACE INIT ANGLE WITH ZERO IF WE'RE DOING CURRENT BASED ZEROING
    m_hood.SetSelectedSensorPosition(ShooterConstants::hoodInitAngle);
    // m_hoodZero = true;
    // if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
    //     m_hood.Set(ControlMode::PercentOutput, 0.0);
    //     m_hood.SetSelectedSensorPosition(0);
    //     m_hoodZero = true;
    // }

    // if(!m_turretZero){
    //     m_turret.Set(ControlMode::PercentOutput, 0.20);
    // }
    // if(!m_hoodZero){
    //     m_hood.Set(ControlMode::PercentOutput, -0.30);   
    // }
}


//NOTE: SHOULD NOT BE CALLED IN CASE WHERE WE'RE DOING RAISED ZERO HOOD
void
Shooter::zeroHood(){
    if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
        m_hood.Set(ControlMode::PercentOutput, 0.0);
        m_hood.SetSelectedSensorPosition(0);   
        m_hoodZero = true;
    }
    if(!m_hoodZero){
        m_hood.Set(ControlMode::PercentOutput, -0.30);   
    }
}


//Load Function
void
Shooter::Load(){
    if(m_state == State::SHOOT || m_state == State::Tarmac){
        m_kicker.Set(ControlMode::PercentOutput, 0.2);
    } 
    else if(m_state == State::LOAD){
        if(!m_photogate.Get() && !m_photogate2.Get()){
            m_kicker.Set(ControlMode::PercentOutput, 0.0);
            m_channel.setState(Channel::State::IDLE);
        } 
        else if(m_photogate2.Get() && !m_photogate.Get()){
            m_channel.setState(Channel::State::RUN);
            m_kicker.Set(ControlMode::PercentOutput, 0.0);
        }
        else if(!m_photogate2.Get() && m_photogate.Get()){
            m_kicker.Set(ControlMode::PercentOutput, 0.2);
            m_channel.setState(Channel::State::RUN);
        }
        else {
            m_kicker.Set(ControlMode::PercentOutput, 0.2);
            m_channel.setState(Channel::State::RUN);
        }
    }
}


//set the State of the shooter
void
Shooter::setState(State newState){
    m_state = newState;
}


//Check if the turret is aimed
bool
Shooter::Aimed(){
    bool flywheelReady = abs(m_flywheelMaster.GetSelectedSensorVelocity() - m_speed) < 400;
    bool hoodReady = abs(m_hood.GetSelectedSensorPosition() - m_angle) < 100; // make this interval smaller
    bool turretReady = abs(m_limelight->getXOff()+4.3) < 2.5;
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
void
Shooter::Calibrate(){
    m_speed = frc::SmartDashboard::GetNumber("speed", 0.0);
    m_angle = frc::SmartDashboard::GetNumber("angle", 0.0);
    frc::SmartDashboard::PutNumber("speed", m_speed);
    frc::SmartDashboard::PutNumber("angle", m_angle);
    
    // m_turrPos = frc::SmartDashboard::GetNumber("turr_pos", 0.0);
    // frc::SmartDashboard::PutNumber("turr_pos", m_turrPos);
}


//Set PID values
void
Shooter::setPID(){
    // double F = frc::SmartDashboard::GetNumber("F", 0.0);
    // frc::SmartDashboard::PutNumber("F", F);
    double P = frc::SmartDashboard::GetNumber("P", 0.0);
    frc::SmartDashboard::PutNumber("P", P);
    double I = frc::SmartDashboard::GetNumber("I", 0.0);
    frc::SmartDashboard::PutNumber("I", I);
    double D = frc::SmartDashboard::GetNumber("D", 0.0);
    frc::SmartDashboard::PutNumber("D", D);

    // m_turretPos.SetPID(P, I, D);
    m_turret.Config_kP(0, P);
    m_turret.Config_kI(0, I);
    m_turret.Config_kD(0, D);

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
    m_turret.Set(0);
    m_flywheelSlave.Set(0);
    m_flywheelMaster.Set(0);
    m_kicker.Set(0);
    m_hood.Set(0);
    // m_limelight->setLEDMode("OFF");
    m_channel.setState(Channel::State::IDLE);
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


void
Shooter::DisableMotors(){
    m_kicker.Disable();
    m_hood.Disable();
    m_flywheelMaster.Disable();
    m_flywheelSlave.Disable();
    m_channel.DisableMotor();
}


void
Shooter::setColor(bool isblue){
    m_blue = isblue;
    ballColor = m_blue? redBall : blueBall;
}


void
Shooter::LowShot(){
    m_hood.Set(ControlMode::Position, 1500);
    m_flywheelMaster.Set(ControlMode::PercentOutput, 0.5);
    m_flywheelSlave.Set(ControlMode::PercentOutput, -0.5);
}


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


void
Shooter::enablelimelight(){
    m_limelight->setLEDMode("ON");
}