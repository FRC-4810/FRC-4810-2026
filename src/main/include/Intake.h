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
        STATE_AGITATE = 6,
        STATE_MANUAL_INTAKE = 7,
        STATE_MANUAL_OUTTAKE = 8,
        STATE_ERROR = 99
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_MANUAL_RAISE,
        COMMAND_MANUAL_LOWER,
        COMMAND_AUTO_RAISE,
        COMMAND_AUTO_LOWER,
        COMMAND_AGITATE,
        COMMAND_MANUAL_INTAKE,
        COMMAND_MANUAL_OUTTAKE,
        COMMAND_STOP
    };

    // Timeout Constants
    constexpr double dManualRaiseTimeout = 4.0;
    constexpr double dManualLowerTimeout = 2.0;

    constexpr double dAutoRaiseTimeout = 2.0;
    constexpr double dAutoLowerTimeout = 1.0;
    constexpr double dAgitateTimeout = 1.0;

    constexpr double dManualIntakeTimeout = 5.0;
    constexpr double dManualOuttakeTimeout = 5.0;

    // Motor Speed Constants TODO
    constexpr double dManualLowerSpeed = 0.4;
    constexpr double dManualRaiseSpeed = -0.4;
    constexpr double dAutoLowerSpeed = 0.4;
    constexpr double dAutoRaiseSpeed = -0.4;
    constexpr double dAgitateSpeed = -0.4;

    constexpr double dManualIntakeSpeed = 1;
    constexpr double dManualOuttakeSpeed = -1;

    // Setpoint Constants TODO
    constexpr double dUpperLimitSetpoint = 0.0;
    constexpr double dLowerLimitSetpoint = 0.5;
    constexpr double dCenterSetpoint = 0.25;

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
        inline void Agitate()
            { m_eCommand = intake::COMMAND_AGITATE; }
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
        inline bool IsAgitating()
            { return(m_eState == intake::eState::STATE_AGITATE); }
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