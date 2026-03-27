//---------------------------------------------------------------------------
//
// MainStateMachine.h - Main State Machine Class Definition
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 10-Jan-24    JJB     Initial Version.
// 03-Mar-24    GMS     Added Homing State
//
//---------------------------------------------------------------------------

#ifndef MAIN_STATE_MACHINE_H_
#define MAIN_STATE_MACHINE_H_

#include <frc/XboxController.h>

#include "RobotIO.h"

#include "Drivetrain.h"                // Drivetrain state machine class
                                       //    definition
#include "Intake.h"                    // Intake state machine class
                                       //    definition
#include "Magazine.h"                  // Magazine state machine class
                                       //    definition
#include "BasicTurret.h"               // Turret state machine class
                                       //    definition
#include "Shooter.h"                   // Shooter state machine class
                                       //    definition

#include "Shooter.h"                   // Shooter state machine class

#include "Turret.h"

// *------------------------------------------------*
// * Top Level (Main) State Machine Enumerated Type *
// *------------------------------------------------*

namespace RobotMain
{
   enum eState
   {
      STATE_START = 0,
      STATE_IDLE = 1,
      STATE_INTAKE_MANUAL_LOWER = 2,
      STATE_INTAKE_MANUAL_RAISE = 3,
      STATE_INTAKE_AUTO_LOWER = 4,
      STATE_INTAKE_AUTO_RAISE = 5,
      STATE_INTAKE_AGITATE = 6,  //-GMS - Do we need this?
      STATE_INTAKE_RUN_IN = 7,
      STATE_INTAKE_RUN_OUT = 8,
      STATE_SHOOTING_RAMP_UP = 9,
      STATE_SHOOTING = 10,
      STATE_MAGAZINE_MANUAL_OUT = 11,
      STATE_MANUAL_TURRET_ROTATING_LEFT = 12,
      STATE_MANUAL_TURRET_ROTATING_RIGHT = 13,
      STATE_ERROR = 99
   };

   enum eDriveState  
   {
      STATE_NORMAL = 0,
   };

   // Joystick Threshold Values. Defines the value at which the forward
   // push of the joystick triggers an event when a joystick is being
   // used as a button trigger 

   static constexpr double SETPOINT_RIGHT_JOYSTICK_FORWARD_UPPER_THRESHOLD = -0.8;
   static constexpr double SETPOINT_RIGHT_JOYSTICK_FORWARD_LOWER_THRESHOLD = -0.4;
   static constexpr double SETPOINT_RIGHT_JOYSTICK_BACKWARD_LOWER_THRESHOLD = 0.4;
   static constexpr double SETPOINT_RIGHT_JOYSTICK_BACKWARD_UPPER_THRESHOLD = 0.8;

   static constexpr double SETPOINT_LEFT_JOYSTICK_FORWARD_UPPER_THRESHOLD = -0.8;
   static constexpr double SETPOINT_LEFT_JOYSTICK_FORWARD_LOWER_THRESHOLD = -0.4;
   static constexpr double SETPOINT_LEFT_JOYSTICK_BACKWARD_LOWER_THRESHOLD = 0.4;
   static constexpr double SETPOINT_LEFT_JOYSTICK_BACKWARD_UPPER_THRESHOLD = 0.8;

   static constexpr double SETPOINT_RIGHT_JOYSTICK_LEFT_UPPER_THRESHOLD = -0.8;
   static constexpr double SETPOINT_RIGHT_JOYSTICK_LEFT_LOWER_THRESHOLD = -0.4;
   static constexpr double SETPOINT_RIGHT_JOYSTICK_RIGHT_LOWER_THRESHOLD = 0.4;
   static constexpr double SETPOINT_RIGHT_JOYSTICK_RIGHT_UPPER_THRESHOLD = 0.8;
}

class MainStateMachine
{
   public:

      // Constructor/Destructor.

      MainStateMachine();
      ~MainStateMachine()
         { }

      // Class Methods.

      void Initialize( RobotIO *p_pRobotIO );
      void UpdateStatus();
      void Execute();

      Drivetrain m_Drivetrain;
      Intake m_Intake;
   private:
      RobotMain::eState m_eState;      // Current main state
      RobotIO *m_pRobotIO;             // Pointer to Robot I/O Class Instance
      RobotMain::eDriveState m_eDriveState;      // Current Drive state

      //-GMS - Intake agitate timer
      frc::Timer *m_pAgitateTimer;

      // State Machine Object Instances.
      Magazine m_Magazine;
      BasicTurret m_Turret;
      Shooter m_Shooter;
};

#endif // MAIN_STATE_MACHINE_H_
