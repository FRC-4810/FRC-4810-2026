#pragma once

#include "RobotIO.h"
#include <frc/Timer.h>

namespace shooter
{
    enum eState
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_LOW_POWER_RAMP_UP = 2,
        STATE_HIGH_POWER_RAMP_UP = 3,
        STATE_SHOOT = 5,
        STATE_TRACKING_SHOOT = 6,
        STATE_ERROR = 99
    };

    enum eCommand 
    {
        COMMAND_NONE,
        COMMAND_LOW_POWER_SHOOT,
        COMMAND_HIGH_POWER_SHOOT,
        COMMAND_TRACKING_SHOOT,
        COMMAND_STOP
    };

    // Timeout Constants
    static constexpr double dRampUpTimeout = 4.0;
    static constexpr double dShootTimeout = 15.0; //Bring down temperaroly

    // Motor Speed Constants
    static constexpr double dLowPowerRampUpSpeed = 0.625;
    static constexpr double dHighPowerRampUpSpeed = 0.80;
    static constexpr double dFeederSpeed = 0.2;

    // Shooter Velocity Setpoints
    static constexpr double dMediumPowerVelocitySetpoint = 68.0;
    static constexpr double dLowPowerVelocitySetpoint = 40.0;
    static constexpr double dHighPowerVelocitySetpoint = 47.5;

    // Tracking Distance Constants (for interpolation)
    static constexpr double dLowPowerDistance = 2.79;
    static constexpr double dHighPowerDistance = 5.08;

    // Tracking Motor Speed Limits
    static constexpr double dMinPower = 0.3;
    static constexpr double dMaxPower = 0.9;
}



class Shooter
{
public:
    // Constructor/ Deconstructor
    Shooter();
    ~Shooter()
        {   }
    
    // Accessor Methods
    inline bool isIdle()
        { return ( m_eState == shooter::eState::STATE_IDLE ); }
    inline bool isRampingUp()
        { return ( m_eState == shooter::eState::STATE_HIGH_POWER_RAMP_UP || m_eState == shooter::eState::STATE_LOW_POWER_RAMP_UP ); }
    inline bool isShooting()
        { return ( m_eState == shooter::eState::STATE_SHOOT || m_eState == shooter::eState::STATE_TRACKING_SHOOT ); }

    inline void LowPowerShoot()
        { m_eCommand = shooter::eCommand::COMMAND_LOW_POWER_SHOOT; }
    inline void HighPowerShoot()
        { m_eCommand = shooter::eCommand::COMMAND_HIGH_POWER_SHOOT; }
    inline void TrackShoot()
        { m_eCommand = shooter::eCommand::COMMAND_HIGH_POWER_SHOOT; }
    inline void Stop()
        { m_eCommand = shooter::eCommand::COMMAND_STOP; }
    inline void SetDistance( double dTargetDist )
        { dDistanceToTarget = dTargetDist; }

    // Class Methods
    void Initialize( RobotIO *p_pRobotIO );
    void Execute();

private:

    shooter::eState m_eState;
    shooter::eCommand m_eCommand;

    frc::Timer *m_pTimeoutTimer;

    double dDistanceToTarget;

    RobotIO *m_pRobotIO;
};