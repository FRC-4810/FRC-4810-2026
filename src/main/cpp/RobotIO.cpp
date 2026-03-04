//---------------------------------------------------------------------------
//
// RobotIO.cpp
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 17-Feb-22  JJB       Class created.
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


   // *---------------------------------*
   // * Shooter Hardware Initialization * 
   // *---------------------------------*
   configs::TalonFXConfiguration leftShooterMotor_MasterConfig{};

   //leftShooterMotor_MasterConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.5_s ); -GMS - Replace w/ motion magic
   leftShooterMotor_MasterConfig.CurrentLimits.WithSupplyCurrentLimit( 40_A );
   leftShooterMotor_MasterConfig.MotorOutput.Inverted =
      signals::InvertedValue::CounterClockwise_Positive;

   configs::TalonFXConfiguration rightShooterMotor_FollowerConfig{};

   //rightShooterMotor_FollowerConfig.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod( 0.5_s );  -GMS - Replace w/ motion magic
   rightShooterMotor_FollowerConfig.CurrentLimits.WithSupplyCurrentLimit( 40_A );

//-GMS - Motion Magic Configs
   leftShooterMotor_MasterConfig.Voltage.WithPeakForwardVoltage(11_V);   
   leftShooterMotor_MasterConfig.Voltage.WithPeakReverseVoltage(-11_V);   

   leftShooterMotor_MasterConfig.Feedback.WithFeedbackSensorSource(signals::FeedbackSensorSourceValue::RotorSensor);
   leftShooterMotor_MasterConfig.Feedback.WithRotorToSensorRatio( 1.0 );
   leftShooterMotor_MasterConfig.Feedback.WithSensorToMechanismRatio( 1.0 );

   configs::Slot0Configs shooterSlot0Configs{}; //-TODO - tune all these -GMS
   shooterSlot0Configs.kP = 1.0;
   shooterSlot0Configs.kD = 0.0;
   shooterSlot0Configs.kV = 0.0952;
   shooterSlot0Configs.kS = 0.001;
   leftShooterMotor_MasterConfig.WithSlot0(shooterSlot0Configs);

   leftShooterMotor_MasterConfig.MotionMagic.WithMotionMagicAcceleration(200_tr_per_s_sq);   //-Top speed in 0.5s
   leftShooterMotor_MasterConfig.MotionMagic.WithMotionMagicJerk(1000_tr_per_s_cu);          //-Top accel in 0.2s


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
   
}
