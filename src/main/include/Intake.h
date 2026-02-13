#pragma once

#include "RobotIO.h"
#include <frc/Timer.h>

namespace intake 
{
    enum eState
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_MANUAL_RAISE = 2,
        STATE_MANUAL_LOWER = 3,
        STATE_AUTO_RAISE = 4,
        STATE_AUTO_LOWER = 5,
        STATE_MANUAL_INTAKE = 6,
        STATE_MANUAL_OUTTAKE = 7,
        STATE_ERROR
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_MANUAL_RAISE,
        COMMAND_MANUAL_LOWER,
        COMMAND_AUTO_RAISE,
        COMMAND_AUTO_LOWER,
        COMMAND_MANUAL_INTAKE,
        COMMAND_MANUAL_OUTTAKE,
        COMMAND_STOP
    };

    // Timeout Constants
    constexpr double dManualRaiseTimeout = 2.0;
    constexpr double dManualLowerTimeout = 1.0;

    constexpr double dAutoRaiseTimeout = 2.5;
    constexpr double dAutoLowerTimeout = 1.5;

    constexpr double dManualIntakeTimeout = 30.0;
    constexpr double dManualOuttakeTimeout = 30.0;

    // Motor Speed Constants
    constexpr double dManualLowerSpeed = 0.4;
    constexpr double dManualRaiseSpeed = -0.4;
    constexpr double dAutoLowerSpeed = 0.3;
    constexpr double dAutoRaiseSpeed = -0.3;

    constexpr double dManualIntakeSpeed = 0.3;
    constexpr double dManualOuttakeSpeed = -0.3;

}

class Intake 
{
    public:
        // Constructor / Deconstructor
        Intake();
        ~Intake()
            {  }
        
        // Accessor Methods
        inline void Stop()
            { m_eCommand = intake::COMMAND_STOP; }
        inline void ManualRaise()
            { m_eCommand = intake::COMMAND_MANUAL_RAISE; }
        inline void ManualLower()
            { m_eCommand = intake::COMMAND_MANUAL_LOWER; }
        inline void AutoRaise()
            { m_eCommand = intake::COMMAND_AUTO_RAISE; }
        inline void AutoLower()
            { m_eCommand = intake::COMMAND_AUTO_LOWER; }
        inline void ManualIntake()
            { m_eCommand = intake::COMMAND_MANUAL_INTAKE; }
        inline void ManualOuttake()
            { m_eCommand = intake::COMMAND_MANUAL_OUTTAKE; }

        inline bool IsIdle()
            { return(m_eState == intake::eState::STATE_IDLE); }
        inline bool IsManualRaising()
            { return(m_eState == intake::eState::STATE_MANUAL_RAISE); }
        inline bool IsManualLowering()
            { return(m_eState == intake::eState::STATE_MANUAL_LOWER); }
        inline bool IsAutoRaising()
            { return(m_eState == intake::eState::STATE_AUTO_RAISE); }
        inline bool IsAutoLowering()
            { return(m_eState == intake::eState::STATE_AUTO_LOWER); }
        inline bool IsManualIntaking()
            { return(m_eState == intake::eState::STATE_MANUAL_INTAKE); }
        inline bool IsManualOuttaking()
            { return(m_eState == intake::eState::STATE_MANUAL_OUTTAKE); }
        
        // Class Methods
        void Initialize( RobotIO *p_pRobotIO );
        void Execute();

    private:

        intake::eCommand m_eCommand;
        intake::eState m_eState;

        frc::Timer *m_pTimeoutTimer;

        RobotIO *m_pRobotIO;
};