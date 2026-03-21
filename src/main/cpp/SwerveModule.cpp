#include "SwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(
   const ctre::phoenix6::CANBus & canbus, 
   const int driveID,                  // Drive Motor CAN ID
   const int turnID,                   // Turning Motor CAN ID
   const int encoderID )               // Encoder Motor CAN ID
    : m_driveMotor( driveID, canbus ),
      m_turningMotor( turnID, canbus ),
      m_turningEncoder( encoderID, canbus )
{
}

void SwerveModule::ConfigModule()
{
    //Factory Default Configs
    configs::TalonFXConfiguration driveConfig = configs::TalonFXConfiguration{};
    configs::TalonFXConfiguration turnConfig = configs::TalonFXConfiguration{};

    driveConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;

    // The ratio of motor rotor rotations to remote sensor rotations,
    // where a ratio greater than 1 is a reduction.

    // The Talon FX is capable of fusing a remote CANcoder with its rotor
    // sensor to produce a high-bandwidth sensor source.  This feature
    // requires specifying the ratio between the motor rotor and the
    // remote sensor.

    // Note: If this is set to zero, the device will reset back to one.

    // Minimum Value: -1000 - Maximum Value: 1000 - Default Value: 1.0 - Units: scalar

    driveConfig.Feedback.RotorToSensorRatio = 1.0;
    driveConfig.Feedback.SensorToMechanismRatio = swerveModule::kDriveGearRatio;
  
    driveConfig.CurrentLimits.WithStatorCurrentLimit(40_A);
    driveConfig.CurrentLimits.WithSupplyCurrentLimit(40_A);
    driveConfig.CurrentLimits.WithStatorCurrentLimitEnable(true);
    driveConfig.CurrentLimits.WithSupplyCurrentLimitEnable(true);

    turnConfig.CurrentLimits.WithStatorCurrentLimit(40_A);
    turnConfig.CurrentLimits.WithSupplyCurrentLimit(40_A);
    turnConfig.CurrentLimits.WithStatorCurrentLimitEnable(true);
    turnConfig.CurrentLimits.WithSupplyCurrentLimitEnable(true);

    //Drive Onboarded PID Configs
    configs::Slot0Configs driveSlot0Configs{};

    driveSlot0Configs.kS = 0.21; //Output voltage to overcome static friction
    driveSlot0Configs.kV = 1.08; //Output voltage per rotation per second
    driveSlot0Configs.kP = 0.1; //Output voltage per rps of error
    driveSlot0Configs.kI = 0.0; //Output voltage for integral term
    driveSlot0Configs.kD = 0.0; //Output voltage for derivative term

    driveConfig.WithSlot0(driveSlot0Configs);
    m_request.WithSlot(0);

    //Turn Motor Motion Magic configs
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    
    configs::Slot0Configs turnSlot0Configs{};

    turnSlot0Configs.kP = 25;
    turnSlot0Configs.kI = 1;
    turnSlot0Configs.kD = 0;

    turnConfig.WithSlot0(turnSlot0Configs);
    m_turnRequest.WithSlot(0);

    turnConfig.Voltage.PeakForwardVoltage = 11_V;
    turnConfig.Voltage.PeakReverseVoltage = -11_V;

    turnConfig.Feedback = turnConfig.Feedback.WithFusedCANcoder(m_turningEncoder);
    if(m_turningMotor.GetDeviceID() == 2 || m_turningMotor.GetDeviceID() == 4)
    {
        turnConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive; // smth new SSP
    }
    turnConfig.Feedback.RotorToSensorRatio = swerveModule::kTurnGearRatio;

    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 6.5_tps;
    turnConfig.MotionMagic.MotionMagicAcceleration = 65_tr_per_s_sq;
    turnConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t(0.12);
    turnConfig.MotionMagic.MotionMagicExpo_kA = ctre::unit::volts_per_turn_per_second_squared_t(0.1);

    m_driveMotor.SetNeutralMode(signals::NeutralModeValue::Brake);
    m_turningMotor.SetNeutralMode(signals::NeutralModeValue::Brake);

//-JJB - CAN ID 1: Front Left Drive
//-JJB - CAN ID 5: Back Left Drive


    if(m_driveMotor.GetDeviceID() == 3 || m_driveMotor.GetDeviceID() == 7)
    {
        driveConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    }

//-JJB - None of this is necessary if using the FusedCANcoder setup:
/*
    // CANcoder Configurations
    // Need to flip based on CCW or CW
    //-JJB CANcoderConfiguration config = new CANcoderConfiguration();
    //-JJB config.SensorDirection = true; // flip if necessary
    //-JJB encoder.getConfigurator().apply(config);

    // “When you read this angle, treat it as 0°.”

    //-JJB double absolutePosition = encoder.getAbsolutePosition().getValue();
    //-JJB CANcoderConfiguration config = new CANcoderConfiguration();
    //-JJB config.MagnetSensor.MagnetOffset = -absolutePosition;
    //-JJB encoder.getConfigurator().apply(config);


    // Once CANcoders are zeroed, tell Talon FX integrated encoders to match the CANcoder:

    // Get absolute position from CANcoder (0 to 1 rotations)
//-JJB     double absolutePosition = m_turningEncoder.GetAbsolutePosition().GetValueAsDouble();
    units::angle::turn_t absolutePosition = m_turningEncoder.GetAbsolutePosition().GetValue();

    // Convert to motor rotations (adjust for gear ratio!)
    double motorRotations = absolutePosition * swerveModule::kTurnGearRatio;

    // Set TalonFX integrated encoder position
    m_turningMotor.SetPosition(motorRotations);
//-JJB     ctre::phoenix::StatusCode SetPosition(units::angle::turn_t newValue) final
*/
/*
 void SyncIntegratedToAbsolute() {
        // Get absolute position from CANcoder (0 to 1 rotations)
        auto absolutePosition = m_canCoder.GetAbsolutePosition().GetValue();

        // Convert to motor rotations (adjust for gear ratio!)
        constexpr double kAngleGearRatio = 12.8;  // Example SDS Mk4i L2
        double motorRotations = absolutePosition * kAngleGearRatio;

        // Set TalonFX integrated encoder position
        m_angleMotor.SetPosition(motorRotations);
    }
*/







//-JJB - None of this is necessary if using the FusedCANcoder setup:
/*
    // CANcoder Configurations
    // Need to flip based on CCW or CW
    //-JJB CANcoderConfiguration config = new CANcoderConfiguration();
    //-JJB config.SensorDirection = true; // flip if necessary
    //-JJB encoder.getConfigurator().apply(config);

    // “When you read this angle, treat it as 0°.”

    //-JJB double absolutePosition = encoder.getAbsolutePosition().getValue();
    //-JJB CANcoderConfiguration config = new CANcoderConfiguration();
    //-JJB config.MagnetSensor.MagnetOffset = -absolutePosition;
    //-JJB encoder.getConfigurator().apply(config);


    // Once CANcoders are zeroed, tell Talon FX integrated encoders to match the CANcoder:

    // Get absolute position from CANcoder (0 to 1 rotations)
//-JJB     double absolutePosition = m_turningEncoder.GetAbsolutePosition().GetValueAsDouble();
    units::angle::turn_t absolutePosition = m_turningEncoder.GetAbsolutePosition().GetValue();

    // Convert to motor rotations (adjust for gear ratio!)
    double motorRotations = absolutePosition * swerveModule::kTurnGearRatio;

    // Set TalonFX integrated encoder position
    m_turningMotor.SetPosition(motorRotations);
//-JJB     ctre::phoenix::StatusCode SetPosition(units::angle::turn_t newValue) final
*/
/*
 void SyncIntegratedToAbsolute() {
        // Get absolute position from CANcoder (0 to 1 rotations)
        auto absolutePosition = m_canCoder.GetAbsolutePosition().GetValue();

        // Convert to motor rotations (adjust for gear ratio!)
        constexpr double kAngleGearRatio = 12.8;  // Example SDS Mk4i L2
        double motorRotations = absolutePosition * kAngleGearRatio;

        // Set TalonFX integrated encoder position
        m_angleMotor.SetPosition(motorRotations);
    }
*/







    m_driveMotor.GetConfigurator().Apply(driveConfig);
    m_turningMotor.GetConfigurator().Apply(turnConfig);

    m_driveMotor.SetPosition(units::angle::turn_t{0});
}

void SwerveModule::ResetDriveEncoder()
{
    m_driveMotor.SetPosition(units::angle::turn_t{0});
}

void SwerveModule::Stop()
{
    m_driveMotor.StopMotor();
    m_turningMotor.StopMotor();
}

frc::SwerveModuleState SwerveModule::GetState() {
    return {
        units::meters_per_second_t{m_driveMotor.GetVelocity().GetValueAsDouble() * (2.0 * std::numbers::pi * swerveModule::kWheelRadius)},
        units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValueAsDouble() * (std::numbers::pi * 2.0)} // Needs to change - CS
    };
}

// This returns the SwerveModulePosition object of the module, which includes drive distance and turn angle
frc::SwerveModulePosition SwerveModule::GetPosition() {
    
    return {
        units::meter_t{m_driveMotor.GetPosition().GetValueAsDouble() * 2.0 * std::numbers::pi * swerveModule::kWheelRadius},
        -units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValueAsDouble() * (std::numbers::pi * 2.0)}
    };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState)
{
    // Get the current module angle - Encoder returns in rotations, multiply by 2 pi to get radians


    // CS - Change in logic here to fix degree calculations; avoid treating rotations as radians; Temperarory test
    frc::Rotation2d currentAngle =frc::Rotation2d{units::turn_t{
        m_turningMotor.GetPosition().GetValueAsDouble()}}; // GetValue() --> GetValueAsDouble() - SSP

    frc::SwerveModuleState state = desiredState;

    state.Optimize(currentAngle) ;  //Optimize State, avoid spinning over 90 deg


    /*Previous logic 
    frc::Rotation2d currentAngle = frc::Rotation2d{
        units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValue()}};
    */

    // CS - Change Logic again using optomiziation function
    units::turn_t targetTurns = units::turn_t{state.angle.Radians().value() / (2.0 * std::numbers::pi)};
    m_turningMotor.SetControl(m_turnRequest.WithPosition(targetTurns));
    
    /* Previous logic
    m_turningMotor.SetControl(m_turnRequest.WithPosition(
        units::turn_t{(double)state.angle.Radians() / (2 * std::numbers::pi)}));*/


    state.speed *= cos((double)(state.angle.Radians() - currentAngle.Radians())); // -- Removed; CS


    units::turns_per_second_t driveTps =
        units::turns_per_second_t{(double)(state.speed /
            (std::numbers::pi * 2 * swerveModule::kWheelRadius))};

    m_driveMotor.SetControl(m_request.WithVelocity(driveTps));
}