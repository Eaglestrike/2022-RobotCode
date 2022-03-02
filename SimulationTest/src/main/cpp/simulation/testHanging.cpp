#include "testHanging.h"
#include <fstream>
#include "Constants.h"

testHanging::testHanging(Climber & c) : climber (c){
   initTestHanging();
}

void testHanging::initTestHanging(){
    climber.getMotor().Set(ClimbConstants::motorRetractedPose);
}

void testHanging::periodic(){
    if (passed == 2) {
        std::cout << "test passed\n";
        return;
    }
    if (passed == 1) {
        std::cout << "test failed\n";
        return;
    }


}

bool testHanging::hooked(){
    //greater than the threshold so should return true
    PhysicsSim::GetInstance().setFirstTalonCurrent();
    if (climber.hooked() == false){
        return false;
    }
    // less than the threshold so should return false
    PhysicsSim::GetInstance().setFirstTalonCurrent();
    if (climber.hooked() == false){
        return false;
    }



}




void testHanging::waited(){
    
}

void testHanging::transition(){

}

void testHanging::otherTests(){


}
