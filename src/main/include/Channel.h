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