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
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>


using namespace ctre::phoenix6::swerve::requests;

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
   // m_Drivetrain.Initialize( p_pRobotIO );
   m_Intake.Initialize( p_pRobotIO );
   m_Magazine.Initialize( p_pRobotIO );
   m_Turret.Initialize( p_pRobotIO );
   m_Shooter.Initialize( p_pRobotIO );

   m_pAgitateTimer = new frc::Timer();
   m_pAgitateTimer->Reset();
}

//-------------------------------------------------------------------

// Update the status of all object instances associated with this class.

void MainStateMachine::UpdateStatus()
{
   static int i = 0;
   // if(m_Drivetrain.Initialized()) {
      //frc::SmartDashboard::PutNumber("iterator", i++);
   m_Drivetrain.Periodic();
   frc::SmartDashboard::PutBoolean("Drive field relative", driveIsFieldRelative);
   // }
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
         m_Magazine.Execute();
         m_Turret.Execute();
         m_Shooter.Execute();
         m_Intake.Execute();

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
         // *--------------------------------------* - BLC changed to right trigger
         // * Driver Right Trigger - Run Intake in *
         // *--------------------------------------*
         if(m_pRobotIO->m_DriveController.GetRightTriggerAxis() > 0.8)
         {
            m_Intake.ManualIntake();
            m_Intake.Execute();

            m_eState = RobotMain::eState::STATE_INTAKE_RUN_IN;
         }

         // *------------------------------------------* - BLC changed to left trigger
         // * Driver Left Bumper - Run Intake Out/Dump *
         // *------------------------------------------*
         if(m_pRobotIO->m_DriveController.GetLeftTriggerAxis() > 0.8)
         {
            m_Intake.ManualOuttake();
            m_Intake.Execute();
            m_Magazine.RunOut();
            m_Magazine.Execute();

            m_eState = RobotMain::eState::STATE_INTAKE_RUN_OUT;
         }

         // *-------------------------------------------------* - GMS
         // * Operator Left Joystick Up - Manual Intake Raise *
         // *-------------------------------------------------*
         if(m_pRobotIO->m_OperatorController.GetLeftY() < -0.8)   //-TODO - Check this
         {
            m_Intake.ManualRaise();
            m_Intake.Execute();

            m_eState = RobotMain::eState::STATE_INTAKE_MANUAL_RAISE;
         }

         // *---------------------------------------------------* - GMS
         // * Operator Left Joystick Down - Manual Intake Lower *
         // *---------------------------------------------------*
         if(m_pRobotIO->m_OperatorController.GetLeftY() > 0.8)   //-TODO - Check this
         {
            m_Intake.ManualLower();
            m_Intake.Execute();

            m_eState = RobotMain::eState::STATE_INTAKE_MANUAL_LOWER;
         }

         // *------------------------------------------* - GMS
         // * Operator Left Bumper - Auto Intake Raise *
         // *------------------------------------------*
         if(m_pRobotIO->m_OperatorController.GetLeftBumperButton())
         {
            m_Intake.AutoRaise();
            m_Intake.Execute();

            m_eState = RobotMain::eState::STATE_INTAKE_AUTO_RAISE;
         }

         // *-------------------------------------------* - GMS
         // * Operator Right Bumper - Auto Intake Lower *
         // *-------------------------------------------*
         if(m_pRobotIO->m_OperatorController.GetRightBumperButton())
         {
            m_Intake.AutoLower();
            m_Intake.Execute();

            m_eState = RobotMain::eState::STATE_INTAKE_AUTO_LOWER;
         }

         // *------------------------------------------* - GMS
         // * Operator Right Trigger - Low Speed Shoot *
         // *------------------------------------------*
         if(m_pRobotIO->m_OperatorController.GetRightTriggerAxis() > 0.8)
         {
            m_Shooter.LowPowerShoot();
            m_Shooter.Execute();

            m_eState = RobotMain::eState::STATE_SHOOTING_RAMP_UP;
         }

         
         // *------------------------------------------* - GMS
         // * Operator Left Trigger - High Speed Shoot *
         // *------------------------------------------*
         if(m_pRobotIO->m_OperatorController.GetLeftTriggerAxis() > 0.8)
         {
            m_Shooter.HighPowerShoot();
            m_Shooter.Execute();

            m_eState = RobotMain::eState::STATE_SHOOTING_RAMP_UP;
         }

         // *--------------------------------* - GMS
         // * Operator X Button - Hopper Out *
         // *--------------------------------*
         if(m_pRobotIO->m_OperatorController.GetXButton())
         {
            m_Magazine.RunOut();
            m_Magazine.Execute();

            m_eState = RobotMain::eState::STATE_MAGAZINE_MANUAL_OUT;
         }

         // *------------------------------------------------------------*
         // * Operator Right Joystick Right - Manual Rotate Turret Right *
         // *------------------------------------------------------------*

         // Pushing right on the joystick (Y+) produces a positive X value.
         // (it is a CCW rotation around the X axis)

         if ( m_pRobotIO->m_OperatorController.GetRightX() > 
              RobotMain::SETPOINT_RIGHT_JOYSTICK_RIGHT_UPPER_THRESHOLD )
         {
          //  printf( "\n>>> Main - Operator Controller - Manual Rotate Turret Right\n" );

            m_Turret.ManualRotateRight();
            m_Turret.Execute();

         //   printf( "Main - Advancing To Manual Rotating Turret Right\n" );
            m_eState = RobotMain::eState::STATE_MANUAL_TURRET_ROTATING_RIGHT;
         }

         // *----------------------------------------------------------*
         // * Operator Right Joystick Left - Manual Rotate Turret Left *
         // *----------------------------------------------------------*

         // Pushing left on the joystick (Y-) produces a negative X value.
         // (it is a CW rotation around the X axis)

         if ( m_pRobotIO->m_OperatorController.GetRightX() <
              RobotMain::SETPOINT_RIGHT_JOYSTICK_LEFT_UPPER_THRESHOLD )
         {
          //  printf( "\n>>> Main - Operator Controller - Manual Rotate Turret Left\n" );

            m_Turret.ManualRotateLeft();
            m_Turret.Execute();

         //   printf( "Main - Advancing To Manual Rotating Turret Left\n" );
            m_eState = RobotMain::eState::STATE_MANUAL_TURRET_ROTATING_LEFT;
         }
      }

      // *===================================================================*
      // *                                                                   *
      // *                          Command States                           *
      // *                                                                   *
      // *===================================================================*

      // Code to be added

      // ***********************
      // * Intake Run In State *
      // ***********************
      else if(m_eState == RobotMain::eState::STATE_INTAKE_RUN_IN)
      {
         if((!m_pRobotIO->m_DriveController.GetRightTriggerAxis()) > 0.8)
         {
            m_Intake.Stop();
         }

         m_Intake.Execute();

         if(m_Intake.IsIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

      // ***********************
      // * Intake Out In State *
      // ***********************
      else if(m_eState == RobotMain::eState::STATE_INTAKE_RUN_OUT)
      {
         if((!m_pRobotIO->m_DriveController.GetLeftTriggerAxis()) > 0.8)
         {
            m_Intake.Stop();
            m_Magazine.Stop();
         }

         m_Intake.Execute();
         m_Magazine.Execute();

         if(m_Intake.IsIdle() && m_Magazine.IsIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

      // *****************************
      // * Intake Manual Raise State *
      // *****************************
      else if(m_eState == RobotMain::eState::STATE_INTAKE_MANUAL_RAISE)
      {
         if(m_pRobotIO->m_OperatorController.GetLeftY() > -0.8)
         {
            m_Intake.Stop();
         }

         m_Intake.Execute();

         if(m_Intake.IsIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

      // *****************************
      // * Intake Manual Lower State *
      // *****************************
      else if(m_eState == RobotMain::eState::STATE_INTAKE_MANUAL_LOWER)
      {
         if(m_pRobotIO->m_OperatorController.GetLeftY() < 0.8)
         {
            m_Intake.Stop();
         }

         m_Intake.Execute();

         if(m_Intake.IsIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

      // ***************************
      // * Intake Auto Raise State *
      // ***************************
      else if(m_eState == RobotMain::eState::STATE_INTAKE_AUTO_RAISE)
      {
         //One touch - No check if button not pressed

         m_Intake.Execute();

         if(m_Intake.IsIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

      // ***************************
      // * Intake Auto Lower State *
      // ***************************
      else if(m_eState == RobotMain::eState::STATE_INTAKE_AUTO_LOWER)
      {
         //One touch - No check if button not pressed
         
         m_Intake.Execute();

         if(m_Intake.IsIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }
      
      // **************************
      // * Shooting Ramp Up State *
      // **************************
      else if(m_eState == RobotMain::eState::STATE_SHOOTING_RAMP_UP)
      {
         if(m_pRobotIO->m_OperatorController.GetRightTriggerAxis() < 0.8 && m_pRobotIO->m_OperatorController.GetLeftTriggerAxis() < 0.8)
         {
            m_Shooter.Stop();
         }

         m_Shooter.Execute();

         if(m_Shooter.isShooting())
         {
            //m_pRobotIO->m_OperatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.3); //Added Rumble
            m_Magazine.RunIn();
            m_Magazine.Execute();
            m_Intake.ManualIntake();
            m_Intake.Execute();


            m_eState = RobotMain::eState::STATE_SHOOTING;
         }

         if(m_Shooter.isIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
            //m_pRobotIO->m_OperatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0); //Added Rumble
         }
      }

      // ******************
      // * Shooting State *
      // ******************
      else if(m_eState == RobotMain::eState::STATE_SHOOTING)
      {

         //if(m_pRobotIO->IsIntakeLowered())
         //{
           // m_Intake.Agitate();

         //}
         
         if(m_pRobotIO->m_OperatorController.GetRightTriggerAxis() < 0.8 && m_pRobotIO->m_OperatorController.GetLeftTriggerAxis() < 0.8)
         {
            m_Magazine.Stop();
            m_Shooter.Stop();
            m_Intake.Stop();
            m_Turret.Stop();
         } 
         else 
         {
            //m_pRobotIO->m_OperatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.3); //Added Rumble
            // *------------------------------------------------------------*
            // * Operator Right Joystick Right - Manual Rotate Turret Right *
            // *------------------------------------------------------------*

            // Pushing right on the joystick (Y+) produces a positive X value.
            // (it is a CCW rotation around the X axis)

            if ( m_pRobotIO->m_OperatorController.GetRightX() > 
                 RobotMain::SETPOINT_RIGHT_JOYSTICK_RIGHT_UPPER_THRESHOLD )
            {
               //  printf( "\n>>> Main - Operator Controller - Manual Rotate Turret Right\n" );
               m_Turret.ManualRotateRight();
               m_Turret.Execute();
            }

            // *----------------------------------------------------------*
            // * Operator Right Joystick Left - Manual Rotate Turret Left *
            // *----------------------------------------------------------*

            // Pushing left on the joystick (Y-) produces a negative X value.
            // (it is a CW rotation around the X axis)

            else if ( m_pRobotIO->m_OperatorController.GetRightX() <
              RobotMain::SETPOINT_RIGHT_JOYSTICK_LEFT_UPPER_THRESHOLD )
            {
               //  printf( "\n>>> Main - Operator Controller - Manual Rotate Turret Left\n" );

               m_Turret.ManualRotateLeft();
               m_Turret.Execute();

            }
            // Stop if no input
            else
            {
               m_Turret.Stop();
               m_Turret.Execute();
            }
         }  
         
         // *-------------------------------------------------* - BLc
         // * Operator Left Joystick Up - Manual Intake Raise *
         // *-------------------------------------------------*
         if(m_pRobotIO->m_OperatorController.GetLeftY() < -0.8)   //-TODO - Check this
         {
            m_Intake.ManualRaise();
            m_Intake.Execute();

         }

         // *---------------------------------------------------*
         // * Operator Left Joystick Down - Manual Intake Lower *
         // *---------------------------------------------------*
         else if(m_pRobotIO->m_OperatorController.GetLeftY() > 0.8)   //-TODO - Check this
         {
            m_Intake.ManualLower();
            m_Intake.Execute();
         }
         else 
         {
            m_Intake.Stop();
            m_Intake.Execute();
         }
         

         m_Shooter.Execute();
         m_Magazine.Execute();
         m_Intake.Execute();
         m_Turret.Execute();
         

         if(m_Magazine.IsIdle() && m_Shooter.isIdle() && m_Intake.IsIdle())
         {
            //-GMS - If intake is already lowered, we never reached the agitate state (or finished it and lowered manually?), so we don't need to move it. 
            //       Otherwise, we auto lower it back to the bottom position for intaking.
            if(m_pRobotIO->IsIntakeLowered())
            {
               //-GMS - if we are in coast mode, transition to brake
               configs::MotorOutputConfigs configs;
               m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Refresh( configs );
               
               configs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
               m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( configs );

               m_eState = RobotMain::eState::STATE_IDLE;
            }
            else
            {
               //-GMS - will auto transition into brake mode
               m_Intake.AutoLower();
               m_Intake.Execute();

               m_eState = RobotMain::eState::STATE_INTAKE_AUTO_LOWER;
            }
            m_pAgitateTimer->Reset();

            //m_pRobotIO->m_OperatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0); //Added Rumble
         }
      }

      //-TODO - GMS - Add intake raise to agitate rest of balls 

      // *****************************
      // * Magazine Manual Out State *
      // *****************************
      else if(m_eState == RobotMain::eState::STATE_MAGAZINE_MANUAL_OUT)
      {
         if(!m_pRobotIO->m_OperatorController.GetXButton())
         {
            m_Magazine.Stop();
         }

         m_Magazine.Execute();

         if(m_Magazine.IsIdle())
         {
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

      // *------------------------------------*
      // * Manual Turret Rotating Right State *
      // *------------------------------------*

      // The turret is performing an operation.  Remain in this state
      // until the turret has completed the operation (The turret
      // has returned to the idle state), or the operator has released
      // the button.

      else if ( m_eState == RobotMain::eState::STATE_MANUAL_TURRET_ROTATING_RIGHT )
      {
         // printf( "Main - Enter Manual Turret Rotating Right State\n" );

         if ( m_pRobotIO->m_OperatorController.GetRightX() <
                 RobotMain::SETPOINT_RIGHT_JOYSTICK_RIGHT_LOWER_THRESHOLD )
         {
         //   printf( "Main - Manual Turret Rotating Right Stop\n" );
            m_Turret.Stop();
         }

         m_Turret.Execute();

         if ( m_Turret.IsIdle() )
         {
            // printf( "Main - Returning To Idle State\n" );
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

      // *-----------------------------------*
      // * Manual Turret Rotating Left State *
      // *-----------------------------------*

      // The turret is performing an operation.  Remain in this state
      // until the turret has completed the operation (The turret
      // has returned to the idle state), or the operator has released
      // the button.

      else if ( m_eState == RobotMain::eState::STATE_MANUAL_TURRET_ROTATING_LEFT )
      {
         // printf( "Main - Enter Manual Turret Rotating Left State\n" );

         if ( m_pRobotIO->m_OperatorController.GetRightX() >
                 RobotMain::SETPOINT_RIGHT_JOYSTICK_LEFT_LOWER_THRESHOLD )
         {
          //  printf( "Main - Manual Turret Rotating Left Stop\n" );
            m_Turret.Stop();
         }

         m_Turret.Execute();

         if ( m_Turret.IsIdle() )
         {
            // printf( "Main - Returning To Idle State\n" );
            m_eState = RobotMain::eState::STATE_IDLE;
         }
      }

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
         driveIsFieldRelative = !driveIsFieldRelative;
         // m_Drivetrain.ToggleFieldRelative();
      }

      if ( m_pRobotIO->m_DriveController.GetBackButtonPressed() )
      {
         m_Drivetrain.SeedFieldCentric();
      }


      // If driver controller disconects, stop drivetrain and don't run anything else

      if ( ! m_pRobotIO->m_DriveController.IsConnected() )
      {
         m_Drivetrain.Stop();
      }
      else if(m_eDriveState == RobotMain::eDriveState::STATE_NORMAL) // Normal drive state/drive by joysticks
      {
      
         if(m_pRobotIO->m_DriveController.GetXButton()) // Swapped X and B buttons - BLC
         {
            m_Drivetrain.SetControl(RobotCentric{}.WithVelocityX(0_mps).WithVelocityY(0.1_mps));
         }
         else if(m_pRobotIO->m_DriveController.GetBButton())
         {
            m_Drivetrain.SetControl(RobotCentric{}.WithVelocityX(0_mps).WithVelocityY(-0.1_mps));
         }
         else
         {
            /*Drive speeds                                                 1 / Higher is smoother */
            static frc::SlewRateLimiter<units::scalar> xLimiter{1 / 0.5_s}; //Changed from 0.4
            static frc::SlewRateLimiter<units::scalar> yLimiter{1 / 0.5_s};

            double requestedX = xLimiter.Calculate(-m_pRobotIO->m_DriveController.GetLeftY());
            double requestedY = yLimiter.Calculate(-m_pRobotIO->m_DriveController.GetLeftX());
            double requestedOmega = -m_pRobotIO->m_DriveController.GetRightX();
            if (driveIsFieldRelative) {
               m_Drivetrain.SetControl(
                  FieldCentric{}
                     .WithVelocityX(requestedX * MaxSpeed)
                     .WithVelocityY(requestedY * MaxSpeed)
                     .WithRotationalRate(requestedOmega * MaxAngularRate)
                     .WithDeadband(0.16 * MaxSpeed) //accounts for drift
                     .WithRotationalDeadband(0.16 * MaxAngularRate)
               );
            } else {
               m_Drivetrain.SetControl(
                  RobotCentric{}
                     .WithVelocityX(requestedX * MaxSpeed)
                     .WithVelocityY(requestedY * MaxSpeed)
                     .WithRotationalRate(requestedOmega * MaxAngularRate)
                     .WithDeadband(0.1 * MaxSpeed)
                     .WithRotationalDeadband(0.1 * MaxAngularRate)
               );
            }
         }
      }
   }

   else
   {
      //   printf( "Main - Null Robot I/O Pointer Encountered\n" );
   }
   
}
