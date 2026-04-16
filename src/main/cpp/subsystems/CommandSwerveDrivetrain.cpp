#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }

    
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}


void CommandSwerveDrivetrain::LoadPath(std::string pathName, bool resetPose)
{
    try {
        auto path = pathplanner::PathPlannerPath::fromPathFile(pathName);
    //    printf("Loaded path");
        
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

    //    printf("Loaded settings");

        m_currentTrajectory = path->generateTrajectory(
            frc::ChassisSpeeds{}, 
            startPose.Rotation(), 
            robotConfig
        );

        if (resetPose) {
            ResetPose(startPose);
         //   printf("Reset Odometry");
        }

        m_pathTimer.Reset();
        m_pathTimer.Stop();
    } catch (const std::exception& e) {
     //   printf("Failed to load path: %s\n", pathName.c_str());
    }
}

void CommandSwerveDrivetrain::FollowPath()
{
    // Check if trajectory is valid (has duration)
    if (m_currentTrajectory.getTotalTime() <= 0_s) return;

    m_pathTimer.Start();
    units::time::second_t time = m_pathTimer.Get();

    // Get the target state from the trajectory
    pathplanner::PathPlannerTrajectoryState state = m_currentTrajectory.sample(time);

    frc::SmartDashboard::PutNumber("Target X", state.pose.X().value());
    frc::SmartDashboard::PutNumber("Target Y", state.pose.Y().value());

    // Calculate robot-relative speeds using the controller
    auto currentPose = GetState().Pose;
    frc::ChassisSpeeds targetSpeeds = m_pathController.calculateRobotRelativeSpeeds(
        currentPose,
        state
    );

    // auto fieldRelativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(targetSpeeds, currentPose.Rotation());
    // fieldRelativeSpeeds.vy *= -1;
    // fieldRelativeSpeeds.vx *= -1;
    // fieldRelativeSpeeds.omega *= -1;
    // auto backToRobotRelative = frc::ChassisSpeeds::FromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.Rotation());

    // targetSpeeds.vx *= -1;
    // targetSpeeds.omega *= -1;

    SetControl(ctre::phoenix6::swerve::requests::ApplyRobotSpeeds{}.WithSpeeds(targetSpeeds));
}
