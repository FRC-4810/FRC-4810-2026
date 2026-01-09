//---------------------------------------------------------------------------
//
// MainStateMachine.cpp - Main State Machine Class Implementation.
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 16-Feb-25    JJB     Initial Version.
//
//---------------------------------------------------------------------------
//
//==================================================================
//-JJB https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
// WPILib Coordinate System (North - West - Up - (NWU) are positive values
// Axis Rotation follows the right hand rule in the positive directions (NWU).
// When viewed with each positive axis pointing toward you,
// counter-clockwise (CCW) is a positive value and clockwise (CW)
// is a negative value.
//
// In most cases in WPILib programming, 0 degrees is aligned with the positive X axis,
// and 180 degrees is aligned with the negative X axis. CCW rotation is positive, so
// 90 degrees is aligned with the positive Y axis, and -90 degrees is aligned with
// the negative Y axis.
//
// The range is (-180, 180], meaning it is exclusive of -180° and
// inclusive of 180 degrees.
//
//                0
//                X+
//                ^
//                |
//       90 Y+ <--+--> Y- -90
//                |
//                v
//                X-
//               180
//==================================================================
// Joysticks, including the sticks on controllers, don’t use the
// same NWU coordinate system. They use the NED (North-East-Down)
// convention, where the positive X axis points ahead, the positive
// Y axis points right, and the positive Z axis points down. When
// viewed with each positive axis pointing toward you,
// counter-clockwise (CCW) is a positive value and clockwise (CW)
// is a negative value.
//
//                X+
//                ^
//                |
//          Y- <--+--> Y+
//                |
//                v
//                X-
//
// Joystick input values are rotations around an axis, not
// translations. This means:
// - Pushing forward on the joystick (toward the positive X axis)
//   is a CW rotation around the Y axis, giving a negative Y value.
// 
// - Pushing to the right (toward the positive Y axis) is a CCW
//   rotation around the X axis, giving a positive X value.
// 
// - Twisting the joystick CW (toward the positive Y axis) is a
//   CCW rotation around the Z axis, giving a positive Z value.
//==================================================================
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/XboxController.html
// https://forums.firstinspires.org/forum/general-discussions/first-programs/first-robotics-competition/competition-discussion/programming-aa/java-ad/80548-getting-joystick-axis-value-java-wpi-usb-xbox-360-controller
//
// Both Left and Right Joysticks (Sides) on the Controller.
//
// Pushing forward on the joystick (X+) produces a negative Y value.
//    (it is a CW rotation around the Y axis)
//
// double leftYJoystickYValue = m_pRobotIO->m_DriveController.GetLeftY();
//
// Pushing right on the joystick (Y+) produces a positive X value.
//    (it is a CCW rotation around the X axis)
//
// double leftXJoystickXValue = m_pRobotIO->m_DriveController.GetLeftX();
//==================================================================

#include "MainStateMachine.h"          // Main State Machine class definition

MainStateMachine::MainStateMachine()
{
   // printf( "Enter Main State Machine Constructor\n" );

	// Initialize Class Member Variables
   m_pRobotIO = nullptr;
   m_eState = RobotMain::eState::STATE_START;

   m_eDriveState = RobotMain::eDriveState::STATE_NORMAL; //Default to normal drive state
}

//-------------------------------------------------------------------

void MainStateMachine::Initialize(
   RobotIO *p_pRobotIO )
{
   // printf( ">>> Enter - MainStateMachine::Initialize\n" );

   // Save a pointer to the robot I/O object, and pass the pointer to
   // the individual state machines so they can save it as well.

   m_pRobotIO = p_pRobotIO;
   m_Drivetrain.Initialize( p_pRobotIO );
}

//-------------------------------------------------------------------

// Update the status of all object instances associated with this class.

void MainStateMachine::UpdateStatus()
{

}

//-------------------------------------------------------------------

void MainStateMachine::Execute()
{
   // printf( ">>> Enter - MainStateMachine::Execute\n" );

   // Verify that the pointer to the robot I/O instance is not null.
   // Attempting to use the pointer if it is null will crash the program.

   if ( m_pRobotIO != nullptr )
   {
      // *-------------*
      // * Start State *
      // *-------------*

      // There is currently nothing to be done to transition from the
      // Start State, so immediately transition to the Idle State.

      if ( m_eState == RobotMain::eState::STATE_START )
      {
         // printf( "Main - Enter Start State\n" );

         // Call the subsystem execute methods to allow them to advance
         // through the idle and start states.


         // printf( "Main - Advancing To Idle State\n" );
         m_eState = RobotMain::eState::STATE_IDLE;
      }

      // *------------*
      // * Idle State *
      // *------------*

      // Look for triggers for operations that are mutually exclusive (i.e.
      // cannot happen simultaneously because they use the same hardware).

      else if ( m_eState == RobotMain::eState::STATE_IDLE )
      {
         
      }

      // *===================================================================*
      // *                                                                   *
      // *                          Command States                           *
      // *                                                                   *
      // *===================================================================*

      // Code to be added




      // *-------------------------*
      // * Asynchronous Operations *
      // *-------------------------*

      // Call the execute methods of the Asynchronous operations.

      // *--------------------*
      // * Driving Operations *
      // *--------------------*

      // Driver control to toggle field relative. Useful if Gyro gets offset or disconnected.

      if ( m_pRobotIO->m_DriveController.GetStartButtonPressed() )
      {
         m_Drivetrain.ToggleFieldRelative();
      }

      // If driver controller disconects, stop drivetrain and don't run anything else

      if ( ! m_pRobotIO->m_DriveController.IsConnected() )
      {
         m_Drivetrain.Execute(0, 0, 0);
      }
      else if(m_eDriveState == RobotMain::eDriveState::STATE_NORMAL) // Normal drive state/drive by joysticks
      {
      
         if(m_pRobotIO->m_DriveController.GetBButton())
         {
            m_Drivetrain.DriveBotRelative(0, 0.1, 0);
         }
         else if(m_pRobotIO->m_DriveController.GetXButton())
         {
            m_Drivetrain.DriveBotRelative(0, -0.1, 0);
         }
         else
         {
            m_Drivetrain.Execute(m_pRobotIO->m_DriveController.GetLeftY(), m_pRobotIO->m_DriveController.GetLeftX(), m_pRobotIO->m_DriveController.GetRightX());
         }
      }
   }

   else
   {
      printf( "Main - Null Robot I/O Pointer Encountered\n" );
   }
}
