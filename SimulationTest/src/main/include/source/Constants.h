#pragma once


namespace
GeneralConstants{
    const double timeStep = 0.02;

    //In meters
    const double goalHeight = 2.641;
    const double cameraHeight = 0.4572;
    const double cameraPitch = 50;
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

    //Absolute Encoder Offset Values - SwerveDrive.h
    const double FLOFF = 0.43;
    const double FROFF = 0.695;
    const double BLOFF = 0.78;
    const double BROFF = 0.19;

    //Ports for Back Right Swerve Module - SwerveDrive.h
    const int BRanglePort = 18;
    const int BRspeedPort = 17;
    const int BRencoderPortA = 6;
    const int BRencoderPortB = 7;
    const int BRencoderPWM = 13;

    //Ports for Back Left Swerve Module - SwerveDrive.h
    const int BLanglePort = 16;
    const int BLspeedPort = 15;
    const int BLencoderPortA = 4;
    const int BLencoderPortB = 5;
    const int BLencoderPWM = 12;

    //Ports for Front Right Swerve Module - SwerveDrive.h
    const int FRanglePort = 14;
    const int FRspeedPort = 13;
    const int FRencoderPortA = 2;
    const int FRencoderPortB = 3;
    const int FRencoderPWM = 11;

    //Ports for Front Left Swerve Module - SwerveDrive.h
    const int FLanglePort = 12;
    const int FLspeedPort = 11;
    const int FLencoderPortA = 0;
    const int FLencoderPortB = 1;
    const int FLencoderPWM = 10;

    //Wheeldrive 
    constexpr double MAX_VOLTS = 1.0;
    const double speedGearRatio = 1/6.12;
    const double angleGearRatio = 1/12.8;
    const double P = 0.012;
    const double I = 0.008;
    const double D = 0.0001;
    const double Pinit = 4.8;
    const double Iinit = 1.2;
    const double Dinit = 0.02;

    //Swerve Controller
    const double fwdstrP = 0.45;
    const double fwdstrI = 0.0;
    const double fwdstrD = 0.0;
    const double rotP = 0.0006;
    const double rotI = 0.00004;
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
    const double turretP = 0.019; //wip
    const double turretI = 0.0006;
    const double turretD = 0.001;

    //Flywheel Controller
    const double flywheelP = 0.0;
    const double flywheelI = 0.0;
    const double flywheelD = 0.0;

    //Hood Controller
    const double hoodP = 0.00013; //wip
    const double hoodI = 0.00011;
    const double hoodD = 0.000007;

    //Turret limit values;
    const double turretMax = -1000.0; //confirmed
    const double turretMin = -63000.0; //confirmed

    const double hoodMax = 5801; //confirmed
    const double hoodMin = 0; //confirmed

    const double angleOff = 300;

    const double zeroingcurrent = 2.5; //confirmed
}

//None of these values have been set yet
namespace
IntakeConstants{
    const int intakeMotorPort = 50; //confirmed

    const int solenoidPort = 0;
}


//None of these values are set yet
namespace
ClimbConstants{
    const int gearboxPort1 = 30; //confirmed
    const int gearboxPort2 = 31; //confirmed
    const int solenoid1Port = 1; 
    const int solenoid2Port = 2;
    const int BrakeSolenoidPort = 3; //what is it?

    //these names are kind of bad, i might change them later
    const double idleEnoughTime = 1;
    const double verticalArmExtendEnoughTime = 1;
    const double verticalArmRetractEnoughTime = 1;
    const double diagonalArmExtendEnoughTime = 1;
    const double diagonalArmRaiseEnoughTime = 1;
    const double diagonalArmRetractEnoughTime = 1;
    const double almostDoneTime = 1;

    const double waitToRaiseVerticalTime = 0; //how long to wait from after retracting to raise diagonal arm to vertical
    const double timeToTestExtension = 0;

    const double hookedCurrent = 0;

    const double acceptablePitch = 0;
    const double deltaPitchTolerance = 0;

    const double diagonalArmExtendWaitTime = 0;
    const double diagonalArmRaiseWaitTime = 0;

    const double motorPoseTolerance = 50;

    const double motorExtendedPose = 10000;
    const double motorTestExtendPose = 90000;
    const double motorRetractedPose = 0;

    const double motorP = 0;
    const double motorI = 0;
    const double motorD = 0;

    const double motorMaxOutput = 0.5;
}


namespace
ChannelConstants{
    const int channelMotorPort = 40; //confirmed
}


//0 is intkae
//1 is disk brake for climber gearbox
// 2 is climb first stage
// 3 is climb 2nd stage