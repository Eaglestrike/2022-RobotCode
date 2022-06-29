/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Constants header file

  This file includes mostly all the constants used within the robot
  They can accessed using namespace depending on the subsystem.
*/


#pragma once


namespace
GeneralConstants{
    const double timeStep = 0.02;

    //In meters
    const double goalHeight = 2.641;
    const double cameraHeight = 0.5;
    const double cameraPitch = 42;

    const double Kdt = 0.02;
    const double MAX_VOLTAGE = 12;
}


namespace
OIConstants{
    //Joystick & xbox input ports
    const int l_joy_port = 0;
    const int r_joy_port = 1;
    const int O_joy_port = 2;
}


namespace
DriveConstants{
    //Dimensions of the Robot - SwerveDrive.h
    const double Width = 29;
    const double Length = 29;

    //Absolute Encoder offsets
    const double FLOFF = 107.9;
    const double FROFF = -161.6;
    const double BLOFF = -10.5 + 180;
    const double BROFF = 18.0 + 180;

    //Ports for Back Right Swerve Module - SwerveDrive.h
    const int BRanglePort = 17;
    const int BRspeedPort = 18;
    const int BRencoder = 8;

    //Ports for Back Left Swerve Module - SwerveDrive.h
    const int BLanglePort = 16;
    const int BLspeedPort = 15;
    const int BLencoder = 42;

    //Ports for Front Right Swerve Module - SwerveDrive.h
    const int FRanglePort = 14;
    const int FRspeedPort = 13;
    const int FRencoder = 62;

    //Ports for Front Left Swerve Module - SwerveDrive.h
    const int FLanglePort = 12;
    const int FLspeedPort = 11;
    const int FLencoder = 10;

    //Wheeldrive 
    constexpr double MAX_VOLTS = 1.0;
    const double speedGearRatio = 1/6.12;
    const double angleGearRatio = 1/12.8;

    const double P = 0.08;
    const double I = 0.1;
    const double D = 0.0;

    const double sP = 0.08;
    const double sI = 0.1;
    const double sD = 0.0;

    //Swerve Controller
    const double fwdstrP = 0.58;
    const double fwdstrI = 0.0;
    const double fwdstrD = 0.0;
    const double rotP = 0.0011;
    const double rotI = 0.00006;
    const double rotD = 0.0;
}


//None of these values have been set yet
namespace
ShooterConstants{

    //TalonFX MotorID
    const int shootMotorPortMaster = 22; //confirmed
    const int shootMotorPortSlave = 23; //confirmed
    const int turretMotorPort = 20; //confirmed
    const int hoodMotorPort = 19; //confirmed
    const int kickerMotorPort = 21; //confirmed

    //Limit Switch
    const int turretLimitSwitch = 9; //confirmed

    //Photogate
    const int photogate = 8; 

    //Turret Controller
    const double turretP = 0.021; //fairly good
    const double turretI = 0.000;
    const double turretD = 0.0001;

    //Turret Position Controller
    const double turretPosP = 0.01;
    const double turretPosI = 0.00001;
    const double turretPosD = 0.015;

    //Flywheel Controller
    // 0.049, 0.045, -0.000014, 0.002
    const double flywheelF = 0.048;
    const double flywheelP = 0.04;
    const double flywheelI = 0.00005;
    const double flywheelD = 0.0015;

    //Hood Controller
    // 0.051, 0.00019, 0.0, 0.0
    const double hoodP = 0.1; 
    const double hoodI = 0.001;
    const double hoodD = 0.03;
    const double hoodF = 0.0;

    //Turret limit values;
    const double turretMax = 0.0; //confirmed
    const double turretMin = -63000.0; //confirmed

    const double hoodMax = 5700; //confirmed
    const double hoodMin = 0; //confirmed

    const double angleOff = 300;

    const double zeroingcurrent = 2.8; //confirmed

    const double TalonFXunitsperrot = 2048;

    const double maxOffset = 18.0;
}


namespace
IntakeConstants{
    const int intakeMotorPort = 40; //confirmed used to be 50?

    const int solenoidPort = 0;
}


//None of these values are set yet
namespace
ClimbConstants{
    const int gearboxPort1 = 30; //confirmed
    const int gearboxPort2 = 31; //confirmed
    const int solenoid1Port = 2;
    const int solenoid2Port = 3;
    const int diskBrakePort = 1;
}


namespace
ChannelConstants{
    const int channelMotorPort = 40; // confirmed
}


//0 is intkae
//1 is disk brake for climber gearbox
// 2 is climb first
// 3 is climb 2nd stage