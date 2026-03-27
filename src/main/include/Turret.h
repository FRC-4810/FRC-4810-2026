#pragma once

#include <frc/Timer.h>

#include "RobotIO.h"
#include "Drivetrain.h"

namespace turret
{
    enum eState
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_HOMING = 2,       //Home is far left rotation
        STATE_DEBOUNCE = 3,
        STATE_MANUAL_LEFT = 4,  //Left is towards Home
        STATE_MANUAL_RIGHT = 5, //Right is away from Home
        STATE_AUTO_MOVE = 6,    //Automatically move to m_dTurretSetpoint
        STATE_FOLLOW = 7,       // Follow the hub with the turret
        STATE_ERROR = 99
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_HOME,
        COMMAND_MANUAL_LEFT,
        COMMAND_MANUAL_RIGHT,
        COMMAND_AUTO_MOVE,
        COMMAND_FOLLOW,
        COMMAND_STOP
    };

    // Timeout Constants
    static constexpr double dHomingTimeout = 10.0;
    static constexpr double dManualMoveTimeout = 10.0;
    static constexpr double dAutoMoveTimeout = 10.0;    //We may get rid of this when we add tracking
    static constexpr double dFollowTimeout = 40.0;    //We may get rid of this when we add tracking
    static constexpr double dDebounceTimeout = 2.0;

    // Motor Speed Constants
    static constexpr double dHomingSpeed = -0.25;
    static constexpr double dManualLeftSpeed = -0.25;
    static constexpr double dManualRightSpeed = 0.25;

    static constexpr double dAutoMoveTollerance = 0.001389;    // Tollerance of plus/minus 0.5 degrees

    static constexpr double dMaxRotations = 0.5;    //-Todo - configure gear ratio to be 0.5
    
}

class Turret
{
public:

    // Constructor/Destructor
    Turret();
    ~Turret()
        {  }

    // Accessor Methods

    inline void Home()
        {  m_eCommand = turret::COMMAND_HOME;  }

    inline void ManualLeft()
        {  m_eCommand = turret::COMMAND_MANUAL_LEFT;  }
    
    inline void ManualRight()
        {  m_eCommand = turret::COMMAND_MANUAL_RIGHT;  }

    inline void SetAutoSetpoint( double dSetpoint )
        {  m_dTurretSetpoint = dSetpoint;  }

    inline void AutoMove()
        {  m_eCommand = turret::COMMAND_AUTO_MOVE;  }

    inline void Stop()
        {  m_eCommand = turret::COMMAND_STOP;  }


    inline bool IsIdle()
        { return(m_eState == turret::eState::STATE_IDLE); }
    
    inline bool IsHoming()
        { return(m_eState == turret::eState::STATE_HOMING); }

    inline bool IsManualLeft()
        { return(m_eState == turret::eState::STATE_MANUAL_LEFT); }

    inline bool IsManualRight()
        { return(m_eState == turret::eState::STATE_MANUAL_RIGHT); }
    
    inline bool IsAutoMoving()
        { return(m_eState == turret::eState::STATE_AUTO_MOVE); }

    //If target - curret is within tollerance 
    inline bool IsAtTarget()
        { return( fabs(m_pRobotIO->m_TurretMotor.GetPosition().GetValueAsDouble() - m_dTurretSetpoint) < turret::dAutoMoveTollerance ); }



    // Class Methods
    void Initialize( RobotIO *p_pRobotIO, Drivetrain *p_pDrivetrain );
    void Execute();
    void UpdateInputStatus();
    
private:
    turret::eState m_eState;
    turret::eCommand m_eCommand;

    double m_dTurretSetpoint;

    Drivetrain *m_pDrivetrain;
    RobotIO *m_pRobotIO;

    frc::Timer *m_pTimeoutTimer;

    configs::MotorOutputConfigs m_MotorConfigs;

    double GetTurretAngleRotations();

    //Motion Magic configs
    controls::MotionMagicVoltage m_Request{0_tr};
};