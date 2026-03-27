//---------------------------------------------------------------------------
//
// RobotIO.cpp
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 17-Feb-22  JJB       Class created.
// 24-Mar-26  CAS       Introduced stator current limits for all other subsystem motors.
//                      Also set peak reverse duty cycle to 0 for the shooter
// 
//---------------------------------------------------------------------------
//
// References:
// com.ctre.phoenix.motorcontrol.can.WPI_TalonFX Class Reference
// https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_f_x.html
//
// com.ctre.phoenix.motorcontrol.can.TalonFX Class Reference
// https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_f_x.html
//
// Notes:
// https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-check-with-motor-drive
// Sensor phase describes the relationship between the motor output
// direction (positive vs negative) and sensor velocity (positive vs
// negative).  For soft-limits and closed-loop features to function
// correctly, the sensor measurement and motor output must be “in-phase”.
//
// Note: Talon FX automatically phases your sensor for you.  It will
// always be correct, provided you use the getSelected* API and have
// configured the selected feedback type to be integrated sensor.
//---------------------------------------------------------------------------

#include "RobotIO.h"                   // Controlled hardware class definition

#include <frc/smartdashboard/SmartDashboard.h>

// https://www.chiefdelphi.com/t/include-wpi-numbers-not-working-after-we-updated-wpilib-to-the-2023-version/424267
// As mentioned at New for 2023 — FIRST Robotics Competition documentation 6, the std numbers header should be used instead (ie #include <numbers>).
// https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html
//-JJB #include <numbers>
#include <numbers>

//-JJB C:\Users\iamro\.gradle\caches\8.11\transforms\be4aaa728fdcee242249e072e2efdf40\transformed\tools-25.1.0-headers\ctre\phoenix6\signals\SpnEnums.hpp

//-JJB - Phoenix V6 API .PDF
//-JJB https://v6.docs.ctr-electronics.com/_/downloads/en/stable/pdf/

#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/spns/SpnValue.hpp"

using namespace ctre::phoenix6;

//-------------------------------------------------------------------

RobotIO::RobotIO()
{
   // printf( "Enter RobotIO Constructor\n" );
}

//-------------------------------------------------------------------

// Initialize all devices on the robot.

void RobotIO::RobotInit()
{
   // *---------------------------------*
   // * Chassis Hardware Initialization *
   // *---------------------------------*

   // Done in SwerveModule.cpp 

   // **********************************
   // * Intake Hardware Initialization *
   // **********************************
   configs::TalonFXConfiguration intakeMoveConfigs{};

   intakeMoveConfigs.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.8_s );
   intakeMoveConfigs.CurrentLimits.WithSupplyCurrentLimit( 30_A );
   intakeMoveConfigs.CurrentLimits.WithSupplyCurrentLimitEnable( true );
   intakeMoveConfigs.CurrentLimits.WithStatorCurrentLimit( 30_A );
   intakeMoveConfigs.CurrentLimits.WithStatorCurrentLimitEnable( true );
   
   intakeMoveConfigs.MotorOutput.Inverted =
      signals::InvertedValue::CounterClockwise_Positive;

   intakeMoveConfigs.Feedback.SensorToMechanismRatio = 25.0;   //-GMS - 25 to 1 gear ratio

   intakeMoveConfigs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast; //-GMS - intake move in coast always


   configs::TalonFXConfiguration intakeRunConfigs{};

   intakeRunConfigs.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.8_s );
   intakeRunConfigs.CurrentLimits.WithSupplyCurrentLimit( 30_A );
   intakeRunConfigs.CurrentLimits.WithStatorCurrentLimitEnable(true);
   intakeRunConfigs.CurrentLimits.WithStatorCurrentLimit( 30_A );
   intakeRunConfigs.CurrentLimits.WithSupplyCurrentLimitEnable(true);
   
   intakeRunConfigs.MotorOutput.Inverted =
      signals::InvertedValue::Clockwise_Positive;

   m_IntakeMoveMotor.GetConfigurator().Apply( intakeMoveConfigs );
   m_IntakeRunMotor.GetConfigurator().Apply( intakeRunConfigs );

   // *----------------------------------*
   // * Magazine Hardware Initialization *
   // *----------------------------------*
   
   configs::TalonFXSConfiguration feederMotorConfigs{};
   
   feederMotorConfigs.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.8_s );
   feederMotorConfigs.CurrentLimits.WithSupplyCurrentLimit( 30_A );
   feederMotorConfigs.CurrentLimits.WithSupplyCurrentLimitEnable(true);
   feederMotorConfigs.CurrentLimits.WithStatorCurrentLimit( 30_A );
   feederMotorConfigs.CurrentLimits.WithStatorCurrentLimitEnable(true); 
   feederMotorConfigs.MotorOutput.WithInverted(signals::InvertedValue::Clockwise_Positive);
   feederMotorConfigs.MotorOutput.WithNeutralMode(signals::NeutralModeValue::Coast);

   feederMotorConfigs.Commutation.WithMotorArrangement(signals::MotorArrangementValue::Minion_JST);

   m_FeederMotor.GetConfigurator().Apply(feederMotorConfigs);

   configs::TalonFXSConfiguration kickerMotorConfigs{};
   
   kickerMotorConfigs.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.8_s );
   kickerMotorConfigs.CurrentLimits.WithSupplyCurrentLimit( 30_A );
   kickerMotorConfigs.CurrentLimits.WithSupplyCurrentLimitEnable(true);
   kickerMotorConfigs.CurrentLimits.WithStatorCurrentLimit( 30_A );
   kickerMotorConfigs.CurrentLimits.WithStatorCurrentLimitEnable(true);
   kickerMotorConfigs.MotorOutput.WithInverted(signals::InvertedValue::Clockwise_Positive);
   kickerMotorConfigs.MotorOutput.WithNeutralMode(signals::NeutralModeValue::Coast);

   kickerMotorConfigs.Commutation.WithMotorArrangement(signals::MotorArrangementValue::Minion_JST);


   m_KickerMotor.GetConfigurator().Apply(kickerMotorConfigs);

   // *--------------------------------*
   // * Turret Hardware Initialization * 
   // *--------------------------------*

   configs::TalonFXSConfiguration turretRotationMotorConfigs{};

   turretRotationMotorConfigs.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.8_s );
   turretRotationMotorConfigs.CurrentLimits.WithSupplyCurrentLimit( 30_A );
   turretRotationMotorConfigs.CurrentLimits.WithSupplyCurrentLimitEnable( true );
   turretRotationMotorConfigs.MotorOutput.WithNeutralMode(signals::NeutralModeValue::Coast);

   m_TurretRotationMotor.GetConfigurator().Apply( turretRotationMotorConfigs );

   // *---------------------------------*
   // * Shooter Hardware Initialization * 
   // *---------------------------------*
   configs::TalonFXConfiguration leftShooterMotor_MasterConfig{};

   leftShooterMotor_MasterConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.5_s );
   leftShooterMotor_MasterConfig.CurrentLimits.WithSupplyCurrentLimit( 40_A );
   leftShooterMotor_MasterConfig.CurrentLimits.WithSupplyCurrentLimitEnable(true);
   leftShooterMotor_MasterConfig.CurrentLimits.WithStatorCurrentLimit( 40_A );
   leftShooterMotor_MasterConfig.CurrentLimits.WithStatorCurrentLimitEnable(true);

   leftShooterMotor_MasterConfig.MotorOutput.Inverted =
      signals::InvertedValue::CounterClockwise_Positive;

   configs::TalonFXConfiguration rightShooterMotor_FollowerConfig{};

   rightShooterMotor_FollowerConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.5_s );
   rightShooterMotor_FollowerConfig.CurrentLimits.WithSupplyCurrentLimit( 40_A );
   rightShooterMotor_FollowerConfig.CurrentLimits.WithSupplyCurrentLimitEnable(true);
   rightShooterMotor_FollowerConfig.CurrentLimits.WithStatorCurrentLimit( 40_A );
   rightShooterMotor_FollowerConfig.CurrentLimits.WithStatorCurrentLimitEnable(true);

   // Apply the Motor Controller Configurations.

   m_LeftShooterMotor_Master.GetConfigurator().Apply( leftShooterMotor_MasterConfig );
   m_RightShooterMotor_Follower.GetConfigurator().Apply( rightShooterMotor_FollowerConfig );

   //-GMS - This is my guess that the shooter will be powered by 2 Kraken Motors on 40A fuses, oposing direction
   m_RightShooterMotor_Follower.SetControl(
      controls::Follower{
         m_LeftShooterMotor_Master.GetDeviceID(),     // Master ID - Device ID of Master to follow
         true});                       // OpposeMasterDirection
                                       // true - Motor invert to oppose the master's
                                       //    configured Invert. Typical when the the
                                       //    master and follower mechanically spin in
                                       //    opposite directions.
}

//-------------------------------------------------------------------

// Update the status of all input devices on the robot.

void RobotIO::UpdateInputStatus()
{
   frc::SmartDashboard::PutNumber("Intake Position", m_IntakeMoveMotor.GetPosition().GetValueAsDouble());
   frc::SmartDashboard::PutBoolean("Intake Lowered", IsIntakeLowered());
   frc::SmartDashboard::PutBoolean("Intake Raised", IsIntakeRaised());
}