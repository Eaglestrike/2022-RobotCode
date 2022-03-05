// #include "testHanging.h"
// #include <fstream>
// #include "Constants.h"
// #include <stdlib.h> 
// #include <cmath>  


// testHanging::testHanging(Climber & c) : climber (c){
//    initTestHanging();
// }

// void testHanging::initTestHanging(){
//     climber.getMotor().Set(ClimbConstants::motorRetractedPose);
//     hookedCurrent = std::abs(ClimbConstants::hookedCurrent);
// }

// void testHanging::periodic(){
//     if (passed == 2) {
//         std::cout << "test passed\n";
//         return;
//     }
//     if (passed == 1) {
//         std::cout << "test failed\n";
//         return;
//     }


// }

// bool testHanging::hooked(){
//     std::cout << "testing if the hooked method works...\n";
//     //for loop to test multiple times with random values
//     for (int i = 0; i < 10; i++) {
//         //greater than the threshold so should return true
//         //I get a random double that is greater than the hooked current (looks confusind but I wasnt sure what specific numbers would work so rand will do)
//         currentRand = (hookedCurrent +  1 +(rand() % (hookedCurrent * 100)) ) /100;
//         PhysicsSim::GetInstance().setFirstTalonCurrent(currentRand);
//         if (climber.hooked() == false){
//             std::cout << "testing hooked failed: hooked was false when the current was far greater...\n";
//             std::cout << "The current which it failed with is:  " << currentRand << "num\n";
//             return false;
//         }
//         // less than the threshold so should return false
//         currentRand = (rand() % (hookedCurrent * 100)) /100;
//         PhysicsSim::GetInstance().setFirstTalonCurrent(currentRand);
//         if (climber.hooked() == true){
//             std::cout << "testing hooked failed: hooked was true when the current was far less...\n";
//             std::cout << "The current which it failed with is:  " << currentRand << "num\n";
//             return false;
//         }

//     }

    

//     return true;

// }




// bool testHanging::waited(){
//     std::cout << "testing waited...\n";
//     // guarantees that it should try to move onto the next state but we do not want that. 
//     PhysicsSim::GetInstance().setFirstTalonCurrent(100000);
//     // gets the start time
//     startTime = frc::Timer().Get();
//     climber.SetState(Climber::State::TEST_DIAGONAL_ARM_EXTEND);
//     currTime = frc::Timer().Get();
//     // will break out if it goes longer than 10 seconds
//     while (currTime - startTime < 10) {
//         currTime = frc::Timer().Get();
//         climber.SetState(Climber::State::TEST_DIAGONAL_ARM_EXTEND);
//         climber.Periodic(0,0,0,false,currTime,false,false,false);
//         if ((climber.GetState()  !=  Climber::State::TEST_DIAGONAL_ARM_EXTEND) &&  (currTime - startTime < timeToTestExtension)) {
//             std::cout << "test failed due to it proceeding to the next state\n";
//             return false;
//         }
//     }
      

    
    
// }

// bool testHanging::otherTests(){
//     // if (climber.getBrake == false){
//     //      std::cout << "failed to break\n";
//     //     return false;
//     // }

// }
