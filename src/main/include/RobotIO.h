//---------------------------------------------------------------------------
//
// RobotIO.h
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 17-Feb-22  JJB       Class created.
//
//---------------------------------------------------------------------------

#pragma once

#include <frc/XboxController.h>        // Driver/Operator Station Controllers

#include <frc/motorcontrol/PWMTalonFX.h>
#include <frc/DigitalInput.h>          // Digital I/O (Photo Eyes, Limit Switches, Encoders)

#include <frc/Servo.h>  //Linear Actuator Testing

//#include "ctre/Phoenix.h"              //-GMS - Old Phoenix 5 controls
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>

//-JJB #include <units/units.h>        // Catch-all.  Use individual headers
#include "units/velocity.h"            // For velocity calculations
#include "units/length.h"              // For distance calculations
#include <units/angle.h>               // For angle representations
#include <units/angular_velocity.h>    // For angular velocity representations

// https://www.chiefdelphi.com/t/include-wpi-numbers-not-working-after-we-updated-wpilib-to-the-2023-version/424267
// As mentioned at New for 2023 — FIRST Robotics Competition documentation 6,
// the std numbers header should be used instead (ie #include <numbers>).
// https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html
//-JJB #include <numbers>
#include <numbers>

using namespace ctre::phoenix6;

class RobotIO
{
   public:

      // Constructor/Destructor.

      RobotIO();
      ~RobotIO()
         { }

      // Device Initialization.

      void RobotInit();
      void UpdateInputStatus();

      // Accessor Methods.

      // *------------------* -GMS
      // * Intake Accessors *
      // *------------------*
      inline bool IsIntakeLowered()
         { return( m_IntakeMoveMotor.GetPosition().GetValueAsDouble() >= 0.55 ); }
      
      //-GMS - check this - intake not raising may be because these need to be inverted
      inline bool IsIntakeRaised()
         { return( m_IntakeLeftLimit.Get() || m_IntakeRightLimit.Get() ); }
      



      // *-------------------* -GMS
      // * Shooter Accessors *
      // *-------------------*
      inline double GetShooterSpeed()
      {
         return( m_LeftShooterMotor_Master.GetVelocity().GetValueAsDouble() );
      }
      

      // Xbox Controllers

      frc::XboxController m_DriveController{ 0 };
      frc::XboxController m_OperatorController{ 1 };

      // *******************
      // * Intake Hardware *
      // *******************
      hardware::TalonFX m_IntakeMoveMotor{ 13 };   //TODO - Check Can ID's
      hardware::TalonFX m_IntakeRunMotor{ 14 };

      frc::DigitalInput m_IntakeLeftLimit{ 8 };
      frc::DigitalInput m_IntakeRightLimit{ 9 };


      // *********************
      // * Magazine Hardware *
      // *********************
      hardware::TalonFXS m_KickerMotor{ 18 };
      hardware::TalonFXS m_FeederMotor{ 19 };

      //-GMS - not yet implemented
      frc::DigitalInput m_EmptyPhotoeye{ 2 };   //Bottom of hopper Photoeye
      frc::DigitalInput m_FullPhotoeye{ 3 };    //Top of hopper Photoeye



      // ******************************  -GMS
      // * Shooter Subsystem Hardware *
      // ******************************
      hardware::TalonFX m_LeftShooterMotor_Master{ 15 };       // Check CAN ID's with rest of Robot Hardware
      hardware::TalonFX m_RightShooterMotor_Follower{ 16 };    // Check CAN ID's with rest of Robot Hardware
      
};
