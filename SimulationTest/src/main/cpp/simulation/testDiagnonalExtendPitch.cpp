#include "testDiagonalExtendPitch.h"

testDiagonalExtendPitch::testDiagonalExtendPitch(Climber& climber) : climber{climber}
{
    init();
}

void testDiagonalExtendPitch::init()
{
    climber.SetState(Climber::State::DIAGONAL_ARM_EXTEND);
    climber.Periodic(0.0, 0.0, 0.0, false, false, false, false, false);
    //climber.getFullExtend().Set(true);
    //climber.getMedExtend().Set(true);  

}

void testDiagonalExtendPitch::periodic() //Yes I should've used a state machine but I kinda went into a braindead trance and it works so I don't wanna change it
{
    //I did't do delays time wise but I don't think I should need them, since the robot should be able to stop itself at any time
    //If I do need them I can add them here though
    
    //I also didn't do anything with motors, since I think that's the least of our problems if we're swinging to such a bad pitch, but such code
    //can be added here relatively easily
    if(passedInitExtend && !passedRetract)
    {
        makePitchBad(true);
    }
    else if(passedRetract && !passedReextend)
    {
        makePitchBad(false);
    }

    if(!passedInitExtend || !passedReextend || !passedRetract)
    {
        test();
    }
    else
    {
        std::cout << "Test passed" << std::endl;
    }
}

void testDiagonalExtendPitch::makePitchBad(bool badPitch)
{
    this->badPitch = badPitch;
    double pitch = (badPitch) ? ClimbConstants::veryBadPitch + 5 : ClimbConstants::veryBadPitch - 5;
    double deltaPitch = (badPitch) ? ClimbConstants::veryBadDeltaPitch + 5 : ClimbConstants::veryBadDeltaPitch - 5;

    climber.Periodic(deltaPitch, pitch, 0.0, false, false, false, false, false);
}

bool testDiagonalExtendPitch::test()
{
    //This code is so repetative it hurts me but it was easy and I don't think making a simulation more efficient is worth the effort
    if(badPitch)
    {
        if(!climber.getFullExtend().Get() || !climber.getMedExtend().Get())
        {
            passedRetract = false;
            std::cout << "Climber still extended with bad pitch - Full Extend: " << std::boolalpha << climber.getFullExtend().Get() << " Med Extend: " << climber.getMedExtend().Get() << std::endl;
        }
        else
        {
            passedRetract = true;
        }
    }
    else if(passedRetract)
    {
        if(climber.getFullExtend().Get() || climber.getMedExtend().Get())
        {
            passedReextend = false;
            std::cout << "Climber not re-extended - Full Extend: " << std::boolalpha << climber.getFullExtend().Get() << " Med Extend: " << climber.getMedExtend().Get() << std::endl;
        }
        else
        {
            passedReextend = true;
        }
    }
    else
    {
        if(climber.getFullExtend().Get() || climber.getMedExtend().Get())
        {
            passedInitExtend = false;
            std::cout << "Climber not initially extended - Full Extend: " << std::boolalpha << climber.getFullExtend().Get() << " Med Extend: " << climber.getMedExtend().Get() << std::endl;
        }
        else
        {
            passedInitExtend = true;
        }
    }

    return false; //Susan: I put this here bc compiler complaining about no return
    
}