#include "gtest/gtest.h"
#include "Shooter.h"
#include "Constants.h"
#include "frc/DigitalInput.h"
#include "frc/simulation/DIOSim.h"
#include <ctre/Phoenix.h>
#include <iostream>
#include <hal/HAL.h>
#include <simulation/PhysicsSim.h>

//NOTE: SINCE CTRE ONLY HAS SIMULATION SUPPORT FOR TALONSRX, TALONS MUST BE DEFINDED AS SRX TO RUN SIMULATION

class TestShooter : public testing::Test {
    protected:
      void Initialize();
      Shooter shooter;
      frc::sim::DIOSim m_turretLimitSwitch{shooter.getLimitSwitch()};
};

void TestShooter::Initialize() {
 PhysicsSim::GetInstance().AddTalonSRX(shooter.getTurretTalon(), 0.75, 3400, false);
}


TEST_F(TestShooter, ZeroNotDoneTest) {
     Initialize();
    m_turretLimitSwitch.SetValue(false); //not tripped
    shooter.Zero();
    PhysicsSim::GetInstance().Run();
    std::cout << "current: " << shooter.getTurretTalon().GetOutputCurrent() << "\n";
    std::cout << "output: " << shooter.getTurretTalon().GetMotorOutputPercent() << "\n";
    EXPECT_DOUBLE_EQ(0.25, shooter.getTurretTalon().GetMotorOutputPercent());
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
