#include <Shooter.h>
#include <iostream>

Shooter::Shooter(){
    colorMatcher.AddColorMatch(red); //red
    colorMatcher.AddColorMatch(blue); //blue

    //m_turret.SetNeutralMode(NeutralMode::Brake);
    //m_hood.SetNeutralMode(NeutralMode::Brake);
    m_flywheelMaster.SetNeutralMode(NeutralMode::Coast);
    m_flywheelSlave.SetNeutralMode(NeutralMode::Coast);
    m_kicker.SetNeutralMode(NeutralMode::Brake);

    //Set the data into the Map
    dataMap[-24.0] = {5800, 18500};
    dataMap[-23.5] = {5750, 18500};
    dataMap[-22.5] = {5700, 18500};
    dataMap[-21.5] = {5600, 18500};
    dataMap[-20.0] = {5500, 18200};
    dataMap[-19.2] = {5500, 18000};
    dataMap[-18.0] = {5400, 17400};
    dataMap[-16.7] = {5400, 17000}; 
    dataMap[-15.4] = {4900, 15700};
    dataMap[-14.5] = {4600, 15500};
    dataMap[-12.6] = {4500, 14500};
    dataMap[-11.2] = {4400, 13800};
    dataMap[-9.7] = {4400, 13600};
    dataMap[-8.0] = {4200, 13500};
    dataMap[-6.2] = {4000, 13500};
    dataMap[-5.0] = {3600, 13500};
    dataMap[-2.5] = {3000, 13900};
    dataMap[-0.5] = {2400, 13200};
    dataMap[5.5] = {1900, 12300};
    dataMap[9.1] = {1700, 12300};
    dataMap[12.4] = {1500, 12300};
    dataMap[16.5] = {1300, 12200};
    dataMap[18.7] = {1200, 12000};
    //m_limelight->setLEDMode("OFF");
    m_hoodZero = false;
    m_turretZero = false;

    m_turretController.SetTolerance(ShooterConstants::turretPoseTolerance, ShooterConstants::deltaTurretPoseTolerance);
    m_hoodController.SetTolerance(ShooterConstants::hoodPoseTolerance, ShooterConstants::deltaHoodPoseTolerance);
    m_flywheelController.SetTolerance(ShooterConstants::flywheelSpeedTolerance, ShooterConstants::deltaFlywheelSpeedTolerance);
}   


//Periodic Function
void
Shooter::Periodic(){
    switch(m_state){
        case State::ZERO:
            Zero();
            break;
        case State::IDLE:
            Stop();
            break;
        case State::LOAD:
            Load();
            break;
        case State::AIM:
            Aim();
            break;
        case State::SHOOT:
            Aim();
            Shoot();
            break;
        case State::MANUAL:
            break;
        default:
            break;
    }
    m_channel.Periodic();
}


//Aim Function for Turret
void
Shooter::Aim(){
    m_limelight->setLEDMode("ON");

    //Check if the limelight sees the target
    // double point = m_limelight->calculateDistance();
    // auto data = dataMap.find(point);
    // if(data != dataMap.end()){
    //     m_angle = data->second.first;
    //     m_speed = data->second.second;
    // } else {
    //     double point1, point2;
    //     if(withinRange(dataPoints, point, point1, point2)){
    //         auto data1 = dataMap[point1];
    //         auto data2 = dataMap[point2];
    //         double angle1, speed1;
    //         double angle2, speed2;
    //         angle1 = data1.first;
    //         speed1 = data1.second;
    //         angle2 = data2.first;
    //         speed2 = data2.second;

    //         m_angle = (angle1 + angle2)/2;
    //         m_speed = (speed1 + speed2)/2;
    //     }else{
    //         m_angle = 0;
    //         m_speed = 0;
    //     }
    // }
    frc::SmartDashboard::PutBoolean("targetAquired", m_limelight->targetAquired());

    // This is the turret movement
    double x_off = m_limelight->getXOff();
    double output = -m_turretController.Calculate(x_off);
    frc::SmartDashboard::PutNumber("Turretoutput", output);
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
        //m_turret.Set(ControlMode:::PercentOutput, output);
    }

    //double out = m_flywheelController.Calculate(m_flywheelMaster.GetSelectedSensorVelocity(), m_speed);
    //m_flywheelMaster.Set(ControlMode::Velocity, out);
    //m_flywheelSlave.Set(ControlMode::Velocity, -out);
    //frc::SmartDashboard::PutNumber("flywheelError", m_speed-m_flywheelMaster.GetSelectedSensorVelocity());
    //frc::SmartDashboard::PutNumber("flywheelSpeed", m_flywheelMaster.GetSelectedSensorVelocity());

    // //Setting the flywheel speed)
    //m_flywheelMaster.Set(ControlMode::Velocity, m_flywheelController.Calculate(m_flywheelMaster.GetSelectedSensorVelocity(), m_speed));
    //m_flywheelSlave.Set(ControlMode::Velocity, m_flywheelController.Calculate(m_flywheelSlave.GetSelectedSensorVelocity(), -m_speed));
    //m_hood.Set(m_hoodController.Calculate(m_hood.GetSelectedSensorPosition(), m_angle));
    m_flywheelMaster.Set(ControlMode::PercentOutput, 0.9);
    m_flywheelSlave.Set(ControlMode::PercentOutput, -0.9);
    //double hoodOut = m_hoodController.Calculate(m_hood.GetSelectedSensorPosition(), m_angle);
    //m_hood.Set(hoodOut);
    //frc::SmartDashboard::PutNumber("HoodError", m_angle - m_hood.GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber("HoodAngle", m_hood.GetSelectedSensorPosition());
}


//binary search on shooter point values
bool
Shooter::withinRange(std::vector<double> array, double p, double p1, double p2){
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
    double confidence = 0.0;
    if ((GeneralConstants::matchColor == "BLUE" && colorMatcher.MatchClosestColor(colorSensor.GetColor(), confidence) == red)
        || (GeneralConstants::matchColor == "RED" && colorMatcher.MatchClosestColor(colorSensor.GetColor(), confidence) == blue)) {
            m_flywheelMaster.Set(ControlMode::PercentOutput, 0.2); //so ball can get out but won't get shot
        }

    if(Aimed()){
        m_channel.setState(Channel::State::RUN);
        Load();
    } else {
        m_channel.setState(Channel::State::IDLE);
    }
}


//Zero Function
//fix
void
Shooter::Zero(){
    if(!m_turretLimitSwitch.Get()){
        m_turret.Set(ControlMode::PercentOutput, 0.0);
        m_turret.SetSelectedSensorPosition(0);
        m_turretZero = true;
    }
    if(m_hood.GetSupplyCurrent() >= ShooterConstants::zeroingcurrent){
        m_hood.Set(ControlMode::PercentOutput, 0.0);
        m_hood.SetSelectedSensorPosition(0);
        m_hoodZero = true;
    }

    if(!m_turretZero){
        m_turret.Set(ControlMode::PercentOutput, 0.20);
    }
    if(!m_hoodZero){
        m_hood.Set(ControlMode::PercentOutput, -0.20);   
    }
}


//Load Function
void
Shooter::Load(){
    //never called in shoot?
    // if(m_state == State::SHOOT){
    //     m_kicker.Set(ControlMode::PercentOutput, 0.2);
    // } 
    // else if(m_state == State::LOAD){
    if(!m_photogate.Get()){
        m_kicker.Set(ControlMode::PercentOutput, 0.0);
        m_channel.setState(Channel::State::IDLE);
    } else {
        m_kicker.Set(ControlMode::PercentOutput, 0.2);
        m_channel.setState(Channel::State::RUN);
    }
}
//}


//set the State of the shooter
void
Shooter::setState(State newState){
    m_state = newState;
}


//Check if the turret is aimed and ready to shoot
bool
Shooter::Aimed(){
    // return (abs(m_limelight->getXOff()) <= ShooterConstants::xOffTolerance 
    //     && m_turretController.AtSetpoint() && m_flywheelController.AtSetpoint() && m_hoodController.AtSetpoint());
    // ^^^ the real one ^^^


    //Adjust all of the offset values
    // if(m_limelight->getXOff() <= ShooterConstants::angleOff
    //     && m_flywheelMaster.GetSelectedSensorVelocity() <= 1000
    //     && m_hood.GetSelectedSensorPosition() <= 10){
    //     return true;
    // }
    if(abs(m_flywheelMaster.GetSelectedSensorVelocity()) > 10000 ){
        return true;
    }
    else {
        return false;
    }
}


//Calibrate function
void
Shooter::Calibrate(){
    //Print out Get Select Sensor Velocity, Position, and direction
    // frc::SmartDashboard::PutNumber("hood position",m_hood.GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("Flywheel speed",m_flywheelMaster.GetSelectedSensorVelocity());
    //frc::SmartDashboard::PutNumber("turretposition", m_turret.GetSelectedSensorPosition());
    
    // m_turretController.SetPID(pGain_a, iGain_a, dGain_a);
    m_speed = frc::SmartDashboard::GetNumber("speed", 0.0);
    //m_angle = frc::SmartDashboard::GetNumber("angle", 0.0);
    frc::SmartDashboard::PutNumber("speed", m_speed);
    //frc::SmartDashboard::PutNumber("angle", m_angle);
}


//Set PID values
void
Shooter::setPID(){
    double P = frc::SmartDashboard::GetNumber("P", 0.0);
    frc::SmartDashboard::PutNumber("Shooter P", P);
    double I = frc::SmartDashboard::GetNumber("I", 0.0);
    frc::SmartDashboard::PutNumber("Shooter I", I);
    double D = frc::SmartDashboard::GetNumber("D", 0.0);
    frc::SmartDashboard::PutNumber("Shooter D", D);
    //m_turretController.SetPID(P, I, D);
    m_flywheelController.SetPID(P,I,D);
    //m_hoodController.SetPID(P,I,D);
    //m_flywheelMaster.Config_kP
}


//Stop All shooter movements
void
Shooter::Stop(){
    m_channel.Stop();
    m_turret.Set(ControlMode::PercentOutput, 0);
    m_flywheelSlave.Set(ControlMode::PercentOutput, 0);
    m_flywheelMaster.Set(ControlMode::PercentOutput, 0);
    m_kicker.Set(ControlMode::PercentOutput, 0);
    m_hood.Set(ControlMode::PercentOutput, 0);
    m_limelight->setLEDMode("OFF");
    m_channel.setState(Channel::State::IDLE);
}


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
    m_turret.Set(ControlMode::PercentOutput, turret_rot*0.2);
}


void
Shooter::limelightOFF(){
    m_limelight->setLEDMode("OFF");
}