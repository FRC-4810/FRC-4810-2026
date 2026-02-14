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
#include "Intake.h"                    // BLC - Intake state machine class

// *------------------------------------------------*
// * Top Level (Main) State Machine Enumerated Type *
// *------------------------------------------------*

namespace RobotMain
{
   enum eState
   {
      STATE_START = 0,
      STATE_IDLE = 1,
      // BLC - Intake states
      STATE_INTAKE_MANUAL_RAISE = 2,
      STATE_INTAKE_MANUAL_LOWER = 3,
      STATE_INTAKE_AUTO_RAISE = 4,
      STATE_INTAKE_AUTO_LOWER = 5,
      STATE_INTAKE_MANUAL_INTAKE = 6,
      STATE_INTAKE_MANUAL_OUTTAKE = 7,

      STATE_ERROR = 99
   };

   // Different states for the drivetrain. Independent to avoid locking
   // driver out of drive controls.  Normal state is regular, states 1
   // & 2 are for reef, states 3 & 4 are for source stations.

   enum eDriveState  
   {
      STATE_NORMAL = 0,
   };

   // BLC - Intake right joystick manual raise/lower threshold setpoints
   constexpr double dIntakeRightJoystickBackwardThreshold = 0.5;
   constexpr double dIntakeRightJoystickForwardThreshold = -0.5;
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

   private:

      RobotMain::eState m_eState;      // Current main state
      RobotIO *m_pRobotIO;             // Pointer to Robot I/O Class Instance
      RobotMain::eDriveState m_eDriveState;      // Current Drive state

      // State Machine Object Instances.
      Drivetrain m_Drivetrain;

      // BLC - Intake state machine object
      Intake m_Intake;
};

#endif // MAIN_STATE_MACHINE_H_
