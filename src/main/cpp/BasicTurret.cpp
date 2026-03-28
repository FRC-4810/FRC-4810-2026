//---------------------------------------------------------------------------
//
// BasicTurret.cpp - Turret Class Implementation.
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 15-Feb-26    CW      Initial Version.
// 07-Mar-26    JJB     Renamed from Shooter to BasicTurret.
//
//---------------------------------------------------------------------------

#include "BasicTurret.h"

BasicTurret::BasicTurret()
{
  // printf( "Enter Turret Constructor\n" );

   // Initialize Class Member Variables

   m_pRobotIO = nullptr;
   m_eState = turret::eState::STATE_START;
   m_eCommand = turret::COMMAND_NONE;
}

//-------------------------------------------------------------------

void BasicTurret::Initialize(
   RobotIO *p_pRobotIO )
{
  // printf( ">>> Enter - Turret::Initialize\n" );

   m_pRobotIO = p_pRobotIO;

   m_pTimeoutTimer = new frc::Timer();
   m_pTimeoutTimer->Reset();
}

//-------------------------------------------------------------------

void BasicTurret::Execute()
{
   // printf( ">>> Enter - Turret::Execute\n" );

   // Verify that the pointer to the robot I/O instance is not null.
   // Attempting to use the pointer if it is null will crash the program.

   if ( m_pRobotIO != nullptr )
   {
      // *-------------*
      // * Start State *
      // *-------------*

      // There is currently nothing to be done to transition from the
      // Start State, so immediately transition to the Idle State.

      if ( m_eState == turret::eState::STATE_START )
      {
         // printf( ">>> Turret - Enter Start State\n" );

         // printf( ">>> Turret - Moving To Idle State\n" );
         m_eState = turret::eState::STATE_IDLE;
      }

      // *------------*
      // * Idle State *
      // *------------*

      // Note: The transition from the Start State to Idle will not occur
      // until a call to the class Execute Method.  Because the Execute
      // method is called selectively based on commands from the
      // Autonomous Routine or the Driver/Operator, the transition from
      // Start to Idle States does not occur until first command is
      // received.  This imposes a 20ms delay in servicing the command in
      // the Idle State.  To eliminate the delay, the "else if" statement
      // below has been changed so that the transition from Start to Idle
      // happens immediately on the first call to the Execute method
      // before the processing of the first command.

      if ( m_eState == turret::eState::STATE_IDLE )
      {
         // printf( ">>> Turret - Enter Idle State\n" );

         // *-----------------------------*
         // * Manual Rotate Right Command *
         // *-----------------------------*

         if ( m_eCommand == turret::COMMAND_MANUAL_ROTATE_RIGHT )
         {
            // printf( "Turret - Manual Rotate Right Command\n" );
//-JJB - Check Position Before Rotating

            double dSpeed = turret::SETPOINT_TURN_RIGHT_SPEED;
           // printf( "Turret - Rotating Right Speed: [%f]\n", dSpeed );

            // Set the timeout timer to the current time so that all
            // timer reads are relative to the time right now.

            m_pTimeoutTimer->Reset();
            m_pTimeoutTimer->Start();

            m_pRobotIO->m_TurretRotationMotor.Set( dSpeed );

          //  printf( "Turret - Moving To Manual Rotating Right State\n" );
            m_eState = turret::eState::STATE_MANUAL_ROTATING_RIGHT;
         }

         // *----------------------------*
         // * Manual Rotate Left Command *
         // *----------------------------*

         else if ( m_eCommand == turret::COMMAND_MANUAL_ROTATE_LEFT )
         {
            // printf( "Turret - Manual Turret Rotate Left Command\n" );
//-JJB - Check Position Before Rotating

            double dSpeed = turret::SETPOINT_TURN_LEFT_SPEED;
           // printf( "Turret - Rotating Left Speed: [%f]\n", dSpeed );

            // Set the timeout timer to the current time so that all
            // timer reads are relative to the time right now.

            m_pTimeoutTimer->Reset();
            m_pTimeoutTimer->Start();

            m_pRobotIO->m_TurretRotationMotor.Set( dSpeed );

            // printf( "Turret - Moving To Manual Rotating Left State\n" );
            m_eState = turret::eState::STATE_MANUAL_ROTATING_LEFT;
         }

         // *-----------------*
         // * Invalid Command *
         // *-----------------*

         else if ( ( m_eCommand != turret::COMMAND_NONE ) &&
                   ( m_eCommand != turret::COMMAND_STOP ) )
         {
          //  printf("BasicTurret: unrecognized command\n");
         }
      }

      // *-----------------------------*
      // * Manual Rotating Right State *
      // *-----------------------------*

      else if ( m_eState == turret::eState::STATE_MANUAL_ROTATING_RIGHT )
      {
         // printf( "Turret - Enter Manual Rotating Right State\n" );

         // Note: The turret can be stopped either by a Stop command
         // issued to the turret by the main state machine, or by the
         // turret because of some condition (reaching limit, timeout,
         // etc.).  Therefore, check the turret termination criteria
         // before calling the execute method, so that the termination
         // can be processed immediately.

         bool bOperationTimeout = false;
         units::time::second_t dCurrentTime = m_pTimeoutTimer->Get();
         // printf( "Turret - Rotate Right Timer Value: [%f]\n", (double)dCurrentTime );

         if ( dCurrentTime >= (units::time::second_t)turret::MANUAL_TURN_RIGHT_TIMEOUT )
         {
            bOperationTimeout = true;
          //  printf( "Turret - Rotating Right Timeout: [%f]\n", (double)dCurrentTime );
         }

         // If the turret has received a stop command, the command
         // timeout has been exceeded, or the upper limit has been
         // reached, then turn off the motor, stop the timer, clear the
         // command, and return to the Idle State.

         // Note: A timeout timer is needed here in case the device fails
         // or communications with the driver station are lost. The
         // timer will prevent getting stuck in this state.

         if ( ( m_eCommand == turret::COMMAND_STOP ) ||
              ( bOperationTimeout ) )
         {
          //  printf( "Turret - Stopping Manual Rotating Right\n" );

            // Stop the motor until set is called again.

            m_pRobotIO->m_TurretRotationMotor.Set( 0.0 );

            // Stop the timer, clear the command, an return to the Idle
            // State.

            m_pTimeoutTimer->Stop();
            m_eCommand = turret::COMMAND_NONE;
          //  printf( "Turret - Returning To Idle State\n" );
            m_eState = turret::eState::STATE_IDLE;
         }
      }

      // *----------------------------*
      // * Manual Rotating Left State *
      // *----------------------------*

      else if ( m_eState == turret::eState::STATE_MANUAL_ROTATING_LEFT )
      {
         // printf( "Turret - Enter Manual Rotating Left State\n" );

         // Note: The turret can be stopped either by a Stop command
         // issued to the turret by the main state machine, or by the
         // turret because of some condition (reaching limit, timeout,
         // etc.).  Therefore, check the turret termination criteria
         // before calling the execute method, so that the termination
         // can be processed immediately.

         bool bOperationTimeout = false;
         units::time::second_t dCurrentTime = m_pTimeoutTimer->Get();
         // printf( "Turret - Rotate Left Timer Value: [%f]\n", (double)dCurrentTime );

         if ( dCurrentTime >= (units::time::second_t)turret::MANUAL_TURN_LEFT_TIMEOUT )
         {
            bOperationTimeout = true;
           // printf( "Turret - Rotating Left Timeout: [%f]\n", (double)dCurrentTime );
         }

         // If the turret has received a stop command, the command
         // timeout has been exceeded, or the upper limit has been
         // reached, then turn off the motor, stop the timer, clear the
         // command, and return to the Idle State.

         // Note: A timeout timer is needed here in case the device fails
         // or communications with the driver station are lost. The
         // timer will prevent getting stuck in this state.

         if ( ( m_eCommand == turret::COMMAND_STOP ) ||
              ( bOperationTimeout ) )
         {
           // printf( "Turret - Stopping Manual Rotating Left\n" );

            // Stop the motor until set is called again.

            m_pRobotIO->m_TurretRotationMotor.Set( 0.0 );

            // Stop the timer, clear the command, an return to the Idle
            // State.

            m_pTimeoutTimer->Stop();
            m_eCommand = turret::COMMAND_NONE;
           // printf( "Turret - Returning To Idle State\n" );
            m_eState = turret::eState::STATE_IDLE;
         }
      }

      // Error or unkown state
      else
      {
         printf("BasicTurret: Error state or unknown state\n");
      }
   }

   // m_pRobotIO is nullptr

   else
   {
      // printf("BasicTurret: m_pRobotIO is nullptr\n");
   }
}
