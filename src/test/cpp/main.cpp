#include "gtest/gtest.h"
#include "Shooter.h"
#include "Constants.h"
#include "frc/DigitalInput.h"
#include "frc/simulation/DIOSim.h"
#include <ctre/Phoenix.h>
#include <iostream>


class TestShooter : public testing::Test {
    protected:
        Shooter shooter;
        frc::sim::DIOSim m_turretLimitSwitch{shooter.getLimitSwitch()};
};

TEST_F(TestShooter, ZeroNotDoneTest) {
    m_turretLimitSwitch.SetValue(false); //not tripped
    shooter.Zero();
    EXPECT_DOUBLE_EQ(0.25, shooter.getFlywheelMaster().GetMotorOutputPercent());
    std::cout << "current: " << shooter.getFlywheelMaster().GetOutputCurrent() << "\n";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
