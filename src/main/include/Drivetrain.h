#pragma once

#include <numbers>
#include <string>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/AnalogGyro.h>

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/CANBus.hpp>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include "SwerveModule.h"

//-GMS - 2026 Auton Setup
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/DriverStation.h>
#include "RobotIO.h"

namespace drivetrain
{
    // Constant Values for max bot linear velocity (Drive Speed) and angular velocity (Spin/turn Speed)

    static constexpr units::meters_per_second_t kMaxSpeed = 4_mps;  // 2.5 meters per second
    static constexpr units::radians_per_second_t kMaxAngularSpeed{ std::numbers::pi };  // 1/2 rotation per second 

    static constexpr units::meter_t kWheelDistance = 0.54782769475136_m;  // The distance bewteen 2 adjacent (like 1 edge of the square, not the diagnol) 
}

using namespace ctre::phoenix6;

class Drivetrain
{
public:
    // Constructor/Destructor.
    Drivetrain();
    ~Drivetrain() 
        { }

    // Class Methods.

    // Accessor Methods.
    inline void ToggleFieldRelative()
        {  m_bIsFieldRelative = !m_bIsFieldRelative;  }

    inline bool IsFieldRelative()
        {  return {m_bIsFieldRelative};  }

    inline frc::Rotation2d GetGyroRotation2d()
        { return{ frc::Rotation2d(units::degree_t{ -m_gyro.GetYaw().GetValue() }) }; }


    
    // ************
    // * Odometry *
    // ************
    
    inline void ResetOdometry()
       { m_poseEstimator.ResetPose( frc::Pose2d{ 0_m, 0_m, frc::Rotation2d(0_deg) } ); }

    void GoToPosition(const frc::Pose2d& targetPose);


    // ***************
    // * PathPlanner *
    // ***************
    void LoadPath(std::string pathName, bool resetPose = false);
    void FollowPath();
    bool IsPathFinished();
    
    void DriveRobotRelative(const frc::ChassisSpeeds& speeds);
    frc::ChassisSpeeds GetRobotRelativeSpeeds();
    inline frc::Pose2d GetBotPose()
       { return( m_poseEstimator.GetEstimatedPosition() ); }

    // Class Methods.

    void Initialize ( RobotIO *p_pRobotIO );

    /**
   * Function that takes in the 3 Joystick Axes and converts them to chassis speeds, then calls 
   * either DriveFieldRelative() or DriveBotRelative() dependent on m_bIsFieldRelative.
   * True - DriveFieldRelative()
   * False - DriveBotRelative()
   *
   * @param leftYJoystickValue The left Y joystick value of the driver controller.
   *     Range -1 to 1.
   * @param leftXJoystickValue The left X joystick value of the driver controller.
   *     Range -1 to 1.
   * @param rightXJoystickValue The right X joystick value of the driver controller.
   *     Range -1 to 1.
   */
    void Execute (  double leftYJoystickValue,          //X speed (forward/backward) - m/s
                    double leftXJoystickValue,          //Y speed (left/right) - m/s
                    double rightXJoystickValue  );      //Rotation Speed - rad/s

    /**
   * Function that drives the robot field-relative, converting inputted speeds into field relative speeds
   * (rotating the vectors) to receive the desired output direction. This is either automatically called by
   * Execute, or can be manually called to bypass all driver speed limiters, slew rate limiters, and the
   * field vs. bot relative boolean 
   *
   * @param xSpeed The X speed of the bot in a field speed (+ is towards opposing driver stations, - is towards own driver station). 
   * Units: meters per second
   * @param ySpeed The Y speed of the bot in a field speed (+ is left from driver perspective, - is right from driver perspective).
   * Units: meters per second
   * @param rotSpeed The rotational speed of the bot (+ is Counter-Clockwise, - is Clockwise)
   * Units: radians per second
   */
    void DriveFieldRelative( double xSpeed, double ySpeed, double rotSpeed );
    
    /**
   * Function that drives the robot bot-relative, inputted speeds stay in respective directions.
   * This is either automatically called by Execute, or can be manually called to bypass all 
   * driver speed limiters, slew rate limiters, and the field vs. bot relative boolean 
   *
   * @param xSpeed The X speed of the bot in a bot speed (+ is towards robot front/forwards, - is towards robot back/backwards). 
   * Units: meters per second
   * @param ySpeed The Y speed of the bot in a bot speed (+ is robot-perspective left, - is robot-perspective right). 
   * Units: meters per second
   * @param rotSpeed The rotational speed of the bot (+ is Counter-Clockwise, - is Clockwise)
   * Units: radians per second
   */
    void DriveBotRelative( double xSpeed, double ySpeed, double rotSpeed );
    
    void Stop();

    ctre::phoenix6::CANBus canbus{"Drivebase"};

//-JJB - 20-Feb-2026 - The encoders were swapped when the new swerve units were install
//-JJB - in the front of the robot.  This change corrects that.
//-JJB     SwerveModule m_frontLeft{canbus,1,2,9};
//-JJB     SwerveModule m_frontRight{canbus,3,4,10};
    SwerveModule m_frontLeft{canbus,1,2,10};
    SwerveModule m_frontRight{canbus,3,4,9};
    SwerveModule m_backLeft{canbus,5,6,11};
    SwerveModule m_backRight{canbus,7,8,12};

private:
    RobotIO *m_pRobotIO;    //Pointer to RobotIO

    hardware::Pigeon2 m_gyro{13,canbus};

    bool m_bIsFieldRelative;    //Field Relative bool (true: field centric; false: bot centric)

    bool m_bLockOnStop; //Lock on stop bool - turn wheels to 45s

    double m_dGyroOffset;

    frc::PIDController m_XController{
        0.0,    //Kp
        0.0,    //Ki
        0.0     //Kd
    };

    frc::PIDController m_YController{
        0.0,    //Kp
        0.0,    //Ki
        0.0     //Kd
    };

    frc::PIDController m_RotController{
        0.0,    //Kp
        0.0,    //Ki
        0.0     //Kd
    };


    //-GMS - updated to 2025 design
    frc::Translation2d m_frontLeftLocation{+drivetrain::kWheelDistance / 2, -drivetrain::kWheelDistance / 2};     //Front Left Wheel Position
    frc::Translation2d m_frontRightLocation{+drivetrain::kWheelDistance / 2, +drivetrain::kWheelDistance / 2};    //Front Right Wheel Position
    frc::Translation2d m_backLeftLocation{-drivetrain::kWheelDistance / 2, -drivetrain::kWheelDistance / 2};      //Back Left Wheel Position
    frc::Translation2d m_backRightLocation{-drivetrain::kWheelDistance / 2, +drivetrain::kWheelDistance / 2};     //Back Right Wheel Position

    // A slew rate limiter is used to limit how fast a signal can change
    // over time.  When controlling velocity or voltage, its main purpose
    // is to prevent sudden, sharp changes that could cause instability,
    // stress, or unsafe behavior.
    // It limits the rate of change (Δoutput / Δtime), not the maximum value.


    // SkewRateLimiter - Limits the rate of change of an input value.
    // Useful for implementing voltage, setpoint, and/or output ramps.
    // A slew-rate limit is most appropriate when the quantity being
    // controlled is a velocity or a voltage. When controlling a position,
    // consider using a TrapezoidProfile instead.

    frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotSpeedLimiter{3 / 1_s};

    // This is an object that is used to go from Bot Speed Values (X,Y,Rot) to 
    // 4 different swerve module states (Turn angle and Speed).
    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation,    
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation
    };


  
    // ************
    // * Odometry *
    // ************

    void UpdateOdometry();  // This might need to be public, probably not

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        m_kinematics,
        GetGyroRotation2d(),
        {
            m_frontLeft.GetPosition(),
            m_frontRight.GetPosition(),
            m_backLeft.GetPosition(),
            m_backRight.GetPosition()
        },
        frc::Pose2d{0_m, 0_m, frc::Rotation2d{units::degree_t{0.0}}}
    };
    
    frc::Timer *m_DrivetrainTimer;
    
    void TryAddVisionMeasurement();
    bool m_bUseCameraMeasurements;

    void UpdatePoseMegatag1();
    void UpdatePoseMegatag2();

    // PathPlanner Members
    pathplanner::PathPlannerTrajectory m_currentTrajectory;
    frc::Timer *m_pathTimer;
    // Controller for following the path
    pathplanner::PPHolonomicDriveController m_pathController{
        pathplanner::PIDConstants{
            5.0,    //Kp
            0.0,    //Ki
            0.0,    //Kd
            0.0     //I-Range
        },
        pathplanner::PIDConstants{
            5.0,    //Kp
            0.0,    //Ki
            0.0,    //Kd
            0.0     //I-Range
        }
    };
};