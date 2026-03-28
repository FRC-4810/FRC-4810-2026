//---------------------------------------------------------------------------
//
// BasicTurret.h - Turret Class Definition.
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 15-Feb-26    CW      Initial Version.
// 07-Mar-26    JJB     Renamed from Shooter to BasicTurret.
//
//---------------------------------------------------------------------------

#pragma once

#include <frc/Timer.h>

#include "RobotIO.h"                   // Controlled hardware class definition

namespace turret
{
   // *-------------------------------*
   // * State Machine Enumerated Type *
   // *-------------------------------*

   // The set of internal states in the state machine implemented by this
   // class.

   enum eState
   {
      STATE_START = 0,
      STATE_IDLE = 1,
      STATE_MANUAL_ROTATING_RIGHT = 2,
      STATE_MANUAL_ROTATING_LEFT = 3,
      STATE_ERROR = 99
   };

   // *-------------------------*
   // * Command Enumerated Type *
   // *-------------------------*

   // The set of external commands that this class supports.

   enum eCommand
   {
      COMMAND_NONE = 0,
      COMMAND_MANUAL_ROTATE_RIGHT,
      COMMAND_MANUAL_ROTATE_LEFT,
      COMMAND_LOW_POWER_SHOOT,
      COMMAND_HIGH_POWER_SHOOT,
      COMMAND_STOP
   };

   // Operation Timeout Timer Setpoints.

   static constexpr units::time::second_t MANUAL_TURN_RIGHT_TIMEOUT = 5.0_s;   // Maximum time to manual turn right
   static constexpr units::time::second_t MANUAL_TURN_LEFT_TIMEOUT = 5.0_s;    // Maximum time to manual turn left

    static constexpr double dRampUpTimeout = 4.0;
    static constexpr double dShootTimeout = 30.0;

   // Operation Speed Percentage Setpoints.

   static constexpr double SETPOINT_TURN_RIGHT_SPEED = 0.10;         // Turn Right Speed
   static constexpr double SETPOINT_TURN_LEFT_SPEED = -0.10;         // Turn Left Speed

   static constexpr double SETPOINT_MIN_ANGLE = -90.0;               // 
   static constexpr double SETPOINT_MAX_ANGLE = 90.0;                // 

   static constexpr double ENCODER_TICKS_PER_DEGREE = 2048.0 /360.0; // TalonFX Calculation
}

class BasicTurret
{
   public:

      // Constructor/Destructor.

      BasicTurret();
      ~BasicTurret()
         { }

      // Accessor Methods.

      inline void ManualRotateRight()
         { m_eCommand = turret::COMMAND_MANUAL_ROTATE_RIGHT; }

      inline void ManualRotateLeft()
         { m_eCommand = turret::COMMAND_MANUAL_ROTATE_LEFT; }

      inline void Stop()
         { m_eCommand = turret::COMMAND_STOP; }

      // Status Accessor Methods.

      inline bool IsIdle()
         { return( m_eState == turret::STATE_IDLE ); }

      inline bool IsRotatingRight()
         { return( m_eState == turret::STATE_MANUAL_ROTATING_RIGHT ); }

      inline bool IsRotatingLeft()
         { return( m_eState == turret::STATE_MANUAL_ROTATING_LEFT ); }

      // Get current turret angle (Degrees).

      inline double GetCurrentAngle() const {
          // Convert TalonFX ticks to degrees
          double dTicks = m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble();
          return ( dTicks / turret::ENCODER_TICKS_PER_DEGREE );
      }

      inline double GetShooterSpeed()
         { return( m_pRobotIO->m_LeftShooterMotor_Master.GetVelocity().GetValueAsDouble() ); }

      // Class Methods.

      void Initialize( RobotIO *p_pRobotIO );
      void Execute();

   private:

      turret::eState m_eState;         // Current state
      RobotIO *m_pRobotIO;             // Pointer to Robot I/O Class Instance

      turret::eCommand m_eCommand;     // Command to perform

      frc::Timer *m_pTimeoutTimer;     // Timeout timer

};