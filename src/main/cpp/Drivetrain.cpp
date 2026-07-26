#include "Drivetrain.h"
#include <frc/filter/SlewRateLimiter.h>

using namespace ctre::phoenix6::swerve::requests;

Drivetrain::Drivetrain() : subsystems::CommandSwerveDrivetrain(
          TunerConstants::DrivetrainConstants,
          TunerConstants::FrontLeft,
          TunerConstants::FrontRight,
          TunerConstants::BackLeft,
          TunerConstants::BackRight)
{
    FieldRelative = true;
}

void Drivetrain::Drive(double joystickX, double joystickY, double joystickRot)
{
    /* Drive speeds:  1 / Higher is smoother */
    static frc::SlewRateLimiter<units::scalar> xLimiter{1 / 0.5_s}; //Changed from 0.4
    static frc::SlewRateLimiter<units::scalar> yLimiter{1 / 0.5_s};

    double requestedX = xLimiter.Calculate(-joystickX);
    double requestedY = yLimiter.Calculate(-joystickY);
    double requestedOmega = -joystickRot;
    if (FieldRelative) {
        SetControl(
            FieldCentric{}
                .WithVelocityX(requestedX * MaxSpeed)
                .WithVelocityY(requestedY * MaxSpeed)
                .WithRotationalRate(requestedOmega * MaxAngularRate)
                .WithDeadband(0.16 * MaxSpeed) //accounts for drift
                .WithRotationalDeadband(0.16 * MaxAngularRate)
        );
    } else {
        SetControl(
            RobotCentric{}
                .WithVelocityX(requestedX * MaxSpeed)
                .WithVelocityY(requestedY * MaxSpeed)
                .WithRotationalRate(requestedOmega * MaxAngularRate)
                .WithDeadband(0.1 * MaxSpeed)
                .WithRotationalDeadband(0.1 * MaxAngularRate)
        );
    }
}

void Drivetrain::BumpLeft(double speed)
{
    SetControl(
        RobotCentric{}
            .WithVelocityX(0.0 * MaxSpeed)
            .WithVelocityY(speed * MaxSpeed)
            .WithRotationalRate(0.0 * MaxAngularRate)
    );
}

void Drivetrain::BumpRight(double speed)
{
    SetControl(
        RobotCentric{}
            .WithVelocityX(0.0 * MaxSpeed)
            .WithVelocityY(-speed * MaxSpeed)
            .WithRotationalRate(0.0 * MaxAngularRate)
    );
}

void Drivetrain::ToggleFieldRelative()
{
    FieldRelative = !FieldRelative;
}