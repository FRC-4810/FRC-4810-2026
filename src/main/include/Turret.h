#pragma once

#include "RobotIO.h"
#include "Drivetrain.h"
#include <frc/Timer.h>
#include <numbers>

namespace turret
{
    enum eState 
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_MANUAL_LEFT = 2,
        STATE_MANUAL_RIGHT = 3,
        STATE_TRACK_HUB = 4,
        STATE_TRACK_CORNER = 5,
        STATE_ERROR = 99
    };

    enum eCommand 
    {
        COMMAND_NONE,
        COMMAND_MANUAL_LEFT,
        COMMAND_MANUAL_RIGHT,
        COMMAND_TRACK_HUB,
        COMMAND_TRACK_CORNER,
        COMMAND_STOP
    };

    // PID Constants - TODO tune this
    static constexpr double Kp = 0.04;
    static constexpr double Ki = 0.0;
    static constexpr double Kd = 0.0;

    // Timeout Constants
    static constexpr double dManualTurnTimeout = 4.0;
    static constexpr double dTrackHubTimeout = 20.0;
    static constexpr double dTrackCornerTimeout = 20.0;

    // Motor Speed Constants
    static constexpr double dManualRightSpeed = -0.1;
    static constexpr double dManualLeftSpeed = 0.1;

    static constexpr double dAutoRightMaxSpeed = -0.1; // TODO - tune this. 0.1 for now to be safe
    static constexpr double dAutoLeftMaxSpeed = 0.1;

    // Setpoint Constants                   - Assumes Zero is at center; This should probably change
    static constexpr double dMinAngleRadians = -1.919; // -110 degrees
    static constexpr double dMaxAngleRadians = 1.919;  // 110 degrees

    static constexpr double dLeftLimitSetpoint = 9.0; // 108 degrees
    static constexpr double dRightLimitSetpoint = -9.0; // -108 degrees

    // Mechanical Constants
    static constexpr double dMotorRotationsPerPiRadians = 15.0;

    static constexpr double dShooterOffset = -0.216; // negative because the shooter is at the back of the robot
                                                  // Distance from the center of the robot to the point of rotation of the turret
                                                  // Should be in meters because Pose2d is in meters

    // Position Constants - not 100% sure if we use blue relative coordinates - change this if needed
    // These are in meters
    static constexpr double dBlueHubX = 4.62; 
    static constexpr double dBlueHubY = 4.02;
    
    static constexpr double dRedHubX = 11.90;
    static constexpr double dRedHubY = 4.02;

    static constexpr double dLeftBlueCornerX = 0.5;
    static constexpr double dLeftBlueCornerY = 1.0;

    static constexpr double dRightBlueCornerX = 0.5;
    static constexpr double dRightBlueCornerY = 7.0;

    static constexpr double dLeftRedCornerX = 16.0;
    static constexpr double dLeftRedCornerY = 1.0;

    static constexpr double dRightRedCornerX = 16.0;
    static constexpr double dRightRedCornerY = 7.0;

    static constexpr double dCenterY = 4.0;

}

class Turret
{
    public:
        // Constuctor/ Deconstructer
        Turret();
        ~Turret()
            {   }
        
        // Accessor Methods
        inline bool IsRotatingManualRight()
            { return( m_eState == turret::eState::STATE_MANUAL_RIGHT ); }
        inline bool IsRotatingManualLeft()
            { return( m_eState == turret::eState::STATE_MANUAL_LEFT ); }
        inline bool IsTrackingHub()
            { return( m_eState == turret::eState::STATE_TRACK_HUB ); }
        inline bool IsTrackingCorner()
            { return( m_eState == turret::eState::STATE_TRACK_CORNER ); }
        inline bool IsIdle()
            { return( m_eState == turret::eState::STATE_IDLE ); }
        
        inline void ManualRight()
            { m_eCommand = turret::eCommand::COMMAND_MANUAL_RIGHT; }
        inline void ManualLeft()
            { m_eCommand = turret::eCommand::COMMAND_MANUAL_LEFT; }
        inline void TrackHub()
            { m_eCommand = turret::eCommand::COMMAND_TRACK_HUB; }
        inline void TrackCorner()
            { m_eCommand = turret::eCommand::COMMAND_TRACK_CORNER; }
        inline void Stop()
            { m_eCommand = turret::eCommand::COMMAND_STOP; }

        double GetTargetDistance();

        // Class Methods
        void Initialize( RobotIO *p_pRobotIO, Drivetrain *p_pDrivetrain );
        void Execute();

    private:
        turret::eCommand m_eCommand;
        turret::eState m_eState;

        frc::Timer *m_pTimeoutTimer;

        RobotIO *m_pRobotIO;

        Drivetrain *m_pDrivetrain; // Include this for Odometry data

        double dTargetX;
        double dTargetY;

        double dShooterX;
        double dShooterY;

        // PID Controller
        double PIDOutput( double dTarget, double dCurrent );

        double m_dErrorSum;
        double m_dLastTime;
        double m_dLastError;

        frc::Timer *m_pPIDTimer;
};