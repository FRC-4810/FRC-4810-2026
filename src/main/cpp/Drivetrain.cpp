#include "Drivetrain.h"
#include "LimelightHelpers.h"

#include <frc/StateSpaceUtil.h>
#include <frc/DriverStation.h>

#include <networktables/NetworkTableInstance.h>

Drivetrain::Drivetrain()

{
    m_bIsFieldRelative = true;

    m_bLockOnStop = false;
    m_dGyroOffset = 0.0;

    m_bUseCameraMeasurements = true;
}

void Drivetrain::Initialize ( RobotIO *p_pRobotIO )
{
    if (initialized == false) {
        m_pRobotIO = p_pRobotIO;

        m_frontLeft.ConfigModule();
        m_frontRight.ConfigModule();
        m_backLeft.ConfigModule();
        m_backRight.ConfigModule();

    //-GMS - Odometry
        m_DrivetrainTimer = new frc::Timer{};
        m_DrivetrainTimer->Reset();
        m_DrivetrainTimer->Start();

        m_pathTimer = new frc::Timer{};
        m_pathTimer->Reset();
        m_pathTimer->Start();

        printf("Drivetrain Initialize\n");

        // Set gyrp offset at start (change m_dGyroOffset depending on what direction the bot should be facing at start)
        m_gyro.SetYaw(units::degree_t{m_dGyroOffset});
        m_gyro.GetYaw().WaitForUpdate(0.2_s); // Make sure pigeon yaw is correct before we set odometry

        LimelightHelpers::SetIMUMode("limelight", 0); //-GMS - Only use Pigeon, ignore Limelight IMU
        //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("setIMUMode", 0);


        //Go To Position PID Controllers
        m_XController.Reset();
        m_XController.SetIZone(0.4);    //-40cm
        m_XController.SetTolerance(0.05);   //-5cm

        m_YController.Reset();
        m_YController.SetIZone(0.4);    //-40cm
        m_YController.SetTolerance(0.05);   //-5cm

        m_RotController.Reset();
        m_RotController.SetIZone(.35);
        m_RotController.SetTolerance(.02);
        m_RotController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);

        initialized = true;
    }
}

void Drivetrain::Execute (
    double leftYJoystickValue,         // Chassis forward/backward
    double leftXJoystickValue,         // Chassis left/right
    double rightXJoystickValue )       // Chassis Rotation (Clockwise/Counterclockwise)
{
    frc::SmartDashboard::PutNumber("Gyro Angle", (double)GetGyroRotation2d().Degrees());
    frc::Pose2d botPose = GetBotPose();
    frc::SmartDashboard::PutNumber("Odometry/Odometry X", (double)botPose.X());
    frc::SmartDashboard::PutNumber("Odometry/Odometry Y", (double)botPose.Y());
    frc::SmartDashboard::PutNumber("Odometry/Odometry Rot", (double)botPose.Rotation().Degrees());

    frc::SmartDashboard::PutBoolean("Drive Field Relative", m_bIsFieldRelative);

    // Left Y value is Chassie X value (forward/backward)
    // Left X value is Chassie Y value (left/right)
    // Right X value is Chassie Rotation value 

//-JJB - This should not be here...
    //-GMS - clamp values so chassis speed never above max speed
    if(pow(leftYJoystickValue, 2) + pow(leftXJoystickValue, 2) > 1 )
    {
        leftYJoystickValue /= sqrt(pow(leftYJoystickValue, 2) + pow(leftXJoystickValue, 2));
        leftXJoystickValue /= sqrt(pow(leftYJoystickValue, 2) + pow(leftXJoystickValue, 2));
    }

//-JJB - This should not be here...
    // All speeds need to be inverted - Correct for joystick inversion.
    const auto xSpeed = m_xSpeedLimiter.Calculate(-frc::ApplyDeadband(leftYJoystickValue, 0.12)) * drivetrain::kMaxSpeed;
    const auto ySpeed = m_ySpeedLimiter.Calculate(frc::ApplyDeadband(leftXJoystickValue, 0.12)) * drivetrain::kMaxSpeed;
    const auto rot =  m_rotSpeedLimiter.Calculate(frc::ApplyDeadband(rightXJoystickValue, 0.12)) * drivetrain::kMaxAngularSpeed;

    //If all speeds are 0 and lock on stop is true, set wheels to opposing 45s
    if((double)xSpeed == 0 && (double)ySpeed == 0 && (double)rot == 0 && m_bLockOnStop)
    {
        m_frontLeft.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{std::numbers::pi / 4}});      //Set Speed to 0, angle to 45
        m_frontRight.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{-std::numbers::pi / 4}});    //Set Speed to 0, angle to -45
        m_backLeft.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{-std::numbers::pi / 4}});      //Set Speed to 0, angle to -45
        m_backRight.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{std::numbers::pi / 4}});      //Set Speed to 0, angle to 45
        
        return;
    }

    if(m_bIsFieldRelative)  //Field Centric Drive
    {
        DriveFieldRelative((double)xSpeed, (double)ySpeed, (double)rot);
    }
    else    //Bot centric drive
    {
        DriveBotRelative((double)xSpeed, (double)ySpeed, (double)rot);
    }
}

void Drivetrain::DriveFieldRelative( double xSpeed, double ySpeed, double rotSpeed )
{
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{xSpeed},
        units::meters_per_second_t{ySpeed},
        units::radians_per_second_t{rotSpeed}, GetGyroRotation2d());

    // Go from speeds to swerve module states using kinematics objects

    auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

    m_frontLeft.SetDesiredState(fl);    //Set Front Left State to target
    m_frontRight.SetDesiredState(fr);   //Set Front Right State to target
    m_backLeft.SetDesiredState(bl);     //Set Back Left State to target
    m_backRight.SetDesiredState(br);    //Set Back Right State to target

    // UpdateOdometry(); // Now called in the periodic
}

void Drivetrain::DriveBotRelative( double xSpeed, double ySpeed, double rotSpeed )
{
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds{
        units::meters_per_second_t{ xSpeed },
        units::meters_per_second_t{ ySpeed },
        units::radians_per_second_t{ rotSpeed }};

    // Go from speeds to swerve module states using kinematics objects

    auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

    m_frontLeft.SetDesiredState(fl);    //Set Front Left State to target
    m_frontRight.SetDesiredState(fr);   //Set Front Right State to target
    m_backLeft.SetDesiredState(bl);     //Set Back Left State to target
    m_backRight.SetDesiredState(br);    //Set Back Right State to target

    // UpdateOdometry(); // Now called in the periodic
}

void Drivetrain::DriveRobotRelative(const frc::ChassisSpeeds& speeds)
{
    auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_backLeft.SetDesiredState(bl);
    m_backRight.SetDesiredState(br);

    // UpdateOdometry(); // Now called in the periodic
}

frc::ChassisSpeeds Drivetrain::GetRobotRelativeSpeeds()
{
    return m_kinematics.ToChassisSpeeds({
        m_frontLeft.GetState(),
        m_frontRight.GetState(),
        m_backLeft.GetState(),
        m_backRight.GetState()
    });
}

void Drivetrain::LoadPath(std::string pathName, bool resetPose)
{
    try {
        auto path = pathplanner::PathPlannerPath::fromPathFile(pathName);
        printf("Loaded path");
        
        // Handle Alliance Flipping
        // CWS - Commenting out alliance flipping for now, as it can be easily handled by flipping the path in the PathPlanner GUI. If we want to add it back in, we should also add a "preventFlipping" boolean to the PathPlannerPath class, and check that here before flipping the path.
        /*auto alliance = frc::DriverStation::GetAlliance();
        if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
            path = path->flipPath();
        }*/

        // Generate the trajectory for this path
        // Using the path's starting rotation and 0 speed (assuming start of auto)
        frc::Pose2d startPose = path->getStartingHolonomicPose().value_or(frc::Pose2d{});
        
        pathplanner::RobotConfig robotConfig = pathplanner::RobotConfig::fromGUISettings(); // Load config from deploy/pathplanner/settings.json

        printf("Loaded settings");

        m_currentTrajectory = path->generateTrajectory(
            frc::ChassisSpeeds{}, 
            startPose.Rotation(), 
            robotConfig
        );

        if (resetPose) {
            ResetOdometry(startPose);
            printf("Reset Odometry");
        }

        m_pathTimer->Reset();
        m_pathTimer->Stop();
    } catch (const std::exception& e) {
        printf("Failed to load path: %s\n", pathName.c_str());
    }
}

void Drivetrain::FollowPath()
{
    // Check if trajectory is valid (has duration)
    if (m_currentTrajectory.getTotalTime() <= 0_s) return;

    m_pathTimer->Start();
    units::time::second_t time = m_pathTimer->Get();

    // Get the target state from the trajectory
    pathplanner::PathPlannerTrajectoryState state = m_currentTrajectory.sample(time);

    frc::SmartDashboard::PutNumber("Target X", state.pose.X().value());
    frc::SmartDashboard::PutNumber("Target Y", state.pose.Y().value());

    // Calculate robot-relative speeds using the controller
    auto currentPose = GetBotPose();
    frc::ChassisSpeeds targetSpeeds = m_pathController.calculateRobotRelativeSpeeds(
        currentPose,
        state
    );

    // auto fieldRelativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(targetSpeeds, currentPose.Rotation());
    // fieldRelativeSpeeds.vy *= -1;
    // fieldRelativeSpeeds.vx *= -1;
    // fieldRelativeSpeeds.omega *= -1;
    // auto backToRobotRelative = frc::ChassisSpeeds::FromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.Rotation());

    targetSpeeds.vx *= -1;
    targetSpeeds.omega *= -1;

    DriveRobotRelative(targetSpeeds);
}

bool Drivetrain::IsPathFinished()
{
    // If trajectory is empty/invalid, consider it finished
    if (m_currentTrajectory.getTotalTime() <= 0_s) return true;

    frc::SmartDashboard::PutNumber("Current time", m_pathTimer->Get().value());
    frc::SmartDashboard::PutNumber("Total time", m_currentTrajectory.getTotalTime().value());

    if ( m_pathTimer->Get() >= m_currentTrajectory.getTotalTime() )
    {
        printf("Path Finished\n");
        return true;
    }
    return false;
}

void Drivetrain::Stop()
{
    m_frontLeft.Stop();
    m_frontRight.Stop();
    m_backLeft.Stop();
    m_backRight.Stop();
}


// ************
// * Odometry *
// ************

void Drivetrain::UpdateOdometry()
{
    //-GMS - Try base odometry code to start - power on in a corner, drive around a room, and return to the 
    // corner. Values for x and y should be near zero. Also check left is positive y, forward is positive x.
    // Driver station should be set to blue alliance, though I don't think it will impact this.
    // Odometry values are posted to shuffleboard under "Odometry" folder.
    double updatedGyroDegrees = (double)-GetGyroRotation2d().Degrees() + 0;
    
    m_poseEstimator.UpdateWithTime(
        frc::Timer::GetFPGATimestamp(),
        frc::Rotation2d(units::degree_t{updatedGyroDegrees}),
        {
            m_frontLeft.GetPositionOdometry(),
            m_frontRight.GetPositionOdometry(),
            m_backLeft.GetPositionOdometry(),
            m_backRight.GetPositionOdometry()
        }
    );
    auto currentPose = m_poseEstimator.GetEstimatedPosition();
    frc::SmartDashboard::PutNumber("Pose X", currentPose.X().value());
    frc::SmartDashboard::PutNumber("Pose Y", currentPose.Y().value());
    frc::SmartDashboard::PutNumber("Pose Theta", currentPose.Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("FL Direction", m_frontLeft.GetPositionOdometry().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("FL Distance", m_frontLeft.GetPositionOdometry().distance.value());

    //Vision Measurements
    TryAddVisionMeasurement();
}

void Drivetrain::CallPeriodic() {
    UpdateOdometry();
}

// Go To Position function
void Drivetrain::GoToPosition(const frc::Pose2d& targetPose) {
    frc::Pose2d currentPose = GetBotPose();

    double xSpeed =  std::clamp(m_XController.Calculate((double)currentPose.X().value(), (double)targetPose.X().value()), (double)-drivetrain::kMaxSpeed, (double)drivetrain::kMaxSpeed);
    double ySpeed =  std::clamp(m_YController.Calculate((double)currentPose.Y().value(), (double)targetPose.Y().value()), (double)-drivetrain::kMaxSpeed, (double)drivetrain::kMaxSpeed);
    double rotSpeed = std::clamp(m_RotController.Calculate((double)currentPose.Rotation().Radians().value(), (double)targetPose.Rotation().Radians().value()), (double)-drivetrain::kMaxAngularSpeed, (double)drivetrain::kMaxAngularSpeed);

    // Drive the robot to position
    DriveFieldRelative(xSpeed, ySpeed, rotSpeed);
}

void Drivetrain::TryAddVisionMeasurement()
{

    //-GMS - Pick which one works best
    //UpdatePoseMegatag1();
    UpdatePoseMegatag2();

}

void Drivetrain::UpdatePoseMegatag1()
{
    LimelightHelpers::SetRobotOrientation("limelight", (double)GetBotPose().Rotation().Degrees(), 0, 0, 0, 0, 0);  //Other 5 values are optional, ommitted for simplicity sake
    
    // ******************
    // * Megatag 1 code *
    // ******************

    LimelightHelpers::PoseEstimate mt1_pose = LimelightHelpers::getBotPoseEstimate_wpiBlue();

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)    //-GMS - Blue alliance
    {
        mt1_pose = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight");
    }
    else    //-GMS - Red alliance
    {
        mt1_pose = LimelightHelpers::getBotPoseEstimate_wpiRed("limelight");
    }

    bool bDoRejectUpdate = false;

    if(mt1_pose.tagCount == 1 && mt1_pose.rawFiducials.size() == 1)
    {
        if(mt1_pose.rawFiducials[0].ambiguity > 0.7)
        {
            bDoRejectUpdate = true;
//            printf("Reject Vision - Ambugity too high\n");
        }

        if(mt1_pose.rawFiducials[0].distToCamera > 3)
        {
            bDoRejectUpdate = true;
//            printf("Reject Vision - Distance to camera too high\n");
        }
    }
    if(mt1_pose.tagCount == 0)
    {
        bDoRejectUpdate = true;
//        printf("Reject Vision - No tags detected\n");
    }

    if(!bDoRejectUpdate)
    {
        m_poseEstimator.SetVisionMeasurementStdDevs({0.5, 0.5, 99999999.0});
        m_poseEstimator.AddVisionMeasurement(
            mt1_pose.pose,
            mt1_pose.timestampSeconds
        );
        printf("Pose Added: X: [%f], Y: [%f], Rot: [%f]\n", (double)mt1_pose.pose.X(), (double)mt1_pose.pose.Y(), (double)mt1_pose.pose.Rotation().Degrees());
    }
}

void Drivetrain::UpdatePoseMegatag2()
{
    // ******************
    // * Megatag 2 code *
    // ******************

    LimelightHelpers::SetRobotOrientation("limelight", m_gyro.GetYaw().GetValueAsDouble(), 0, 0, 0, 0, 0);
    LimelightHelpers::PoseEstimate mt2Pose;
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)    //-GMS - Blue alliance
    {
        mt2Pose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }
    else    //-GMS - Red alliance
    {
        mt2Pose = LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("limelight");
    }
   
    bool bDoRejectUpdate = false;

    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    if(fabs(m_gyro.GetAngularVelocityZDevice().GetValueAsDouble()) > 360)
    {
        bDoRejectUpdate = true;
//        printf("Reject Vision - Angular Velocity too high\n");
    }
    if(mt2Pose.tagCount == 0)
    {
        bDoRejectUpdate = true;
//        printf("Reject Vision - No tags detected\n");
    }

    if(!bDoRejectUpdate)
    {
        if(mt2Pose.tagCount > 1)
        {
            //m_poseEstimator.SetVisionMeasurementStdDevs({0.002, 0.002, 9999999.0});
            ResetOdometry(mt2Pose.pose);
        }
        else
        {
            m_poseEstimator.SetVisionMeasurementStdDevs({0.01, 0.01, 9999999.0});
        }
        m_poseEstimator.AddVisionMeasurement(
            mt2Pose.pose,
            mt2Pose.timestampSeconds
        );
        printf("Pose Added: X: [%f], Y: [%f], Rot: [%f]\n", (double)mt2Pose.pose.X(), (double)mt2Pose.pose.Y(), (double)mt2Pose.pose.Rotation().Degrees());
    }
}