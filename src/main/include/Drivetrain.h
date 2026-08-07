#include "RobotIO.h"

#include "subsystems/CommandSwerveDrivetrain.h"                // Drivetrain state machine class
#include "generated/TunerConstants.h"

using namespace ctre::phoenix6;

class Drivetrain : public subsystems::CommandSwerveDrivetrain {
public:
    Drivetrain();
    ~Drivetrain() {}

    void Drive(double joystickX, double joystickY, double joystickRot);
    void BumpRight(double speed);
    void BumpLeft(double speed);
    void ToggleFieldRelative();
    void Periodic();

private:
    bool FieldRelative;

    units::meters_per_second_t MaxSpeed = 0.70 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.65_tps; // 3/4 of a rotation per second max angular velocity -- .75 to .6 changed
};