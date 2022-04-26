/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  For the hardware between the intake and shooter header file

  *** THIS IS A DEAD CLASS !!! *** 
  Hardware is incorporated within the Intake
*/


#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>


class Channel{
    public:
        enum State{
            IDLE,
            RUN,
            Badidea
        };

    Channel();
    void Periodic();
    void Run();
    void Stop();
    void Reverse();
    void setState(State newState);
    void DisableMotor();

    private:
        WPI_TalonFX m_channel{ChannelConstants::channelMotorPort, "rio"};

        State m_state;
};