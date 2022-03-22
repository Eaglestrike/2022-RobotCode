#pragma once

/*
Coordinate Frames:

   INTAKE
|O ====== O|
|          |     +x ^
|    X     |        |
|          |  +y    |
|O        O|  <-----+


Angles are measured CCW from +x to +y, per right hand standard.
X marks the orign of the robot frame. The robot coordinate frame moves with the
robot. The world frame is equal to the robot frame at t=0, and is fixed.
*/

#define PI M_PI

namespace GeneralConstants {
const double timeStep = 0.02;

// In meters
const double goalHeight = 2.641;
const double cameraHeight = 0.4572;
const double cameraPitch = 50;
}  // namespace GeneralConstants

namespace OIConstants {
// Joystick & xbox input ports
const int l_joy_port = 0;
const int r_joy_port = 1;
const int O_joy_port = 2;
}  // namespace OIConstants

namespace DriveConstants {
// Dimensions of the Robot - SwerveDrive.h
const double Width = 29;
const double Length = 29;

// Absolute Encoder offsets
const double FLOFF = -108.5;
const double FROFF = 157.0;
const double BLOFF = 7.91;
const double BROFF = -18.98;

// Ports for Back Right Swerve Module - SwerveDrive.h
const int BRanglePort = 18;
const int BRspeedPort = 17;
const int BRencoder = 8;

// Ports for Back Left Swerve Module - SwerveDrive.h
const int BLanglePort = 16;
const int BLspeedPort = 15;
const int BLencoder = 42;

// Ports for Front Right Swerve Module - SwerveDrive.h
const int FRanglePort = 14;
const int FRspeedPort = 13;
const int FRencoder = 62;

// Ports for Front Left Swerve Module - SwerveDrive.h
const int FLanglePort = 12;
const int FLspeedPort = 11;
const int FLencoder = 10;

// Wheeldrive
constexpr double MAX_VOLTS = 1.0;
const double speedGearRatio = 1 / 6.12;
const double angleGearRatio = 1 / 12.8;
const double P = 0.005;
const double I = 0.001;
const double D = 0.0002;
const double Pinit = 0.008;
const double Iinit = 0.001;
const double Dinit = 0.0001;

// Swerve Controller
const double fwdstrP = 0.58;
const double fwdstrI = 0.0;
const double fwdstrD = 0.0;
const double rotP = 0.0011;
const double rotI = 0.00006;
const double rotD = 0.0;

const double EMPIRIC_WHEEL_METERS_PER_TICK = 1.0;  // TODO: TUNE
}  // namespace DriveConstants

// None of these values have been set yet
namespace ShooterConstants {

// TalonFX MotorID
const int shootMotorPortMaster = 22;  // confirmed
const int shootMotorPortSlave = 23;   // confirmed
const int turretMotorPort = 20;       // confirmed
const int hoodMotorPort = 19;         // confirmed
const int kickerMotorPort = 21;       // confirmed

// Limit Switch
const int turretLimitSwitch = 9;  // confirmed

// Photogate
const int photogate = 8;

// Turret Controller
const double turretP = 0.022;  // fairly good
const double turretI = 0.0007;
const double turretD = 0.0012;

// Flywheel Controller
const double flywheelF = 0.049;
const double flywheelP = 0.045;
const double flywheelI = -0.000014;
const double flywheelD = 0.002;

// Hood Controller
const double hoodP = 0.051;
const double hoodI = 0.00016;
const double hoodD = 0.0;
const double hoodF = 0.0;

// Turret limit values;
const double turretMax = -1000.0;   // confirmed
const double turretMin = -63000.0;  // confirmed

const double hoodMax = 5700;  // confirmed
const double hoodMin = 0;     // confirmed

const double angleOff = 300;

const double zeroingcurrent = 1.75;  // confirmed

const double TalonFXunitsperrot = 2048;
}  // namespace ShooterConstants

namespace IntakeConstants {
const int intakeMotorPort = 50;  // confirmed

const int solenoidPort = 0;
}  // namespace IntakeConstants

// None of these values are set yet
namespace ClimbConstants {
const int gearboxPort1 = 30;  // confirmed
const int gearboxPort2 = 31;  // confirmed
const int solenoid1Port = 2;
const int solenoid2Port = 3;
const int diskBrakePort = 1;
}  // namespace ClimbConstants

namespace ChannelConstants {
const int channelMotorPort = 40;  // confirmed
}

// 0 is intkae
// 1 is disk brake for climber gearbox
//  2 is climb first
//  3 is climb 2nd stage
