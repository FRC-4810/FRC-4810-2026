//Generated From Template


#pragma once

#include <frc/Timer.h>

#include "RobotIO.h"

namespace magazine
{
    enum eState
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_RUN_IN = 2,
        STATE_RUN_OUT = 3,
        STATE_ERROR = 99
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_RUN_IN,
        COMMAND_RUN_OUT,
        COMMAND_STOP
    };

    static constexpr double dMagazineTimeout = 30.0;

    static constexpr double dKickerMotorInSpeed = 1.0;
    static constexpr double dKickerMotorOutSpeed = -1.0;

    static constexpr double dFeederMotorInSpeed = 1.0;
    static constexpr double dFeederMotorOutSpeed = -1.0;
    
}

class Magazine
{
public:

    // Constructor/Destructor
    Magazine();
    ~Magazine()
        {  }

    // Accessor Methods

    inline void RunIn()
        {  m_eCommand = magazine::COMMAND_RUN_IN;  }

    inline void RunOut()
        {  m_eCommand = magazine::COMMAND_RUN_OUT;  }

    inline void Stop()
        {  m_eCommand = magazine::COMMAND_STOP;  }




    inline bool IsIdle()
        { return(m_eState == magazine::eState::STATE_IDLE); }

    inline bool IsRunningIn()
        { return(m_eState == magazine::eState::STATE_RUN_IN); }

    inline bool IsRunningOut()
        { return(m_eState == magazine::eState::STATE_RUN_OUT); }



    // Class Methods
    void Initialize( RobotIO *p_pRobotIO );
    void Execute();
    void UpdateInputStatus();
    
private:
    magazine::eState m_eState;
    magazine::eCommand m_eCommand;

    RobotIO *m_pRobotIO;

    frc::Timer *m_pTimeoutTimer;
};