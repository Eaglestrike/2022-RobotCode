//intellisense is complaining but it builds so ¯\_(ツ)_/¯
//#include <hal/HAL.h>
#include "gtest/gtest.h"
#include <Swerve.h>
#include <iostream>

int main(int argc, char** argv) {
 // HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

class SwerveTest : public testing::Test {
    protected:
        Swerve swerve;
};

 TEST_F(SwerveTest, yeetus) {
     swerve.Periodic(
    units::meters_per_second_t{0},
    units::meters_per_second_t{0},
    units::radians_per_second_t{1},
    units::degree_t{0});
 }