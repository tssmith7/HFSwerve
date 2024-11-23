
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "DataLogger.h"

#include "command/DriveCommands.h"

#include "swerve/Drive.h"
#include "swerve/SwerveConstants.h"

const double DEADBAND = 0.1;

frc2::CommandPtr DriveCommands::JoystickDrive( 
    Drive* d, 
    std::function<double()> xSupplier, 
    std::function<double()> ySupplier, 
    std::function<double()> omegaSupplier)
{
    return frc2::cmd::Run( [d, xSupplier, ySupplier, omegaSupplier] {
        // Apply deadband
        double linearMagnitude = frc::ApplyDeadband<double>( std::hypot( xSupplier(), ySupplier() ), DEADBAND );
        frc::Rotation2d linearDirection{ xSupplier(), ySupplier() };
        double omega = frc::ApplyDeadband<double>( omegaSupplier(), DEADBAND );

        // Apply expo by squaring
        linearMagnitude = linearMagnitude * linearMagnitude;
        omega = std::copysign( omega*omega, omega );

        double Vx = linearMagnitude * units::math::cos( linearDirection.Radians() );
        double Vy = linearMagnitude * units::math::sin( linearDirection.Radians() );
        
        // Convert to field relative speeds
        bool isFlipped = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;

        d->RunVelocity( 
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                Vx * swerve::physical::kDriveSpeedLimit,
                Vy * swerve::physical::kDriveSpeedLimit,
                omega * swerve::physical::kTurnSpeedLimit,
                isFlipped ? d->GetRotation() + frc::Rotation2d(180_deg) : d->GetRotation()
            )
        ); 
    },
    {d}
    ).WithName( "Joystick Drive" );
}