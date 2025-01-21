
#include <numbers>
#include <cmath>
#include <units/angle.h>
#include <units/angular_acceleration.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "DataLogger.h"
#include "swerve/SwerveConstants.h"
#include "swerve/Module.h"

// #include "DataLogger.h"


Module::Module( ModuleIO* io, const ModuleConfigs& configs ) :
    io{std::move(io)},
    m_turnPIDController{ configs.turnTune.kP, configs.turnTune.kI, configs.turnTune.kD },
    m_drivePIDController{ configs.driveTune.kP, configs.driveTune.kI, configs.driveTune.kD }
{
    m_driveFF = new frc::SimpleMotorFeedforward<units::radian>{ 
        units::volt_t{configs.driveTune.kS},
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::kv_unit>{configs.driveTune.kV}, 
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::ka_unit>{configs.driveTune.kA}
    };
    m_turnPIDController.EnableContinuousInput( -std::numbers::pi , std::numbers::pi );
    m_name = fmt::format( "/Module{}", configs.index );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void Module::Periodic( ) {
    inputs.processInputs( "Swerve" + m_name );

    // On first cycle reset the motor encoder to the absolute encoder value
    if( !turnRelativeOffset && units::math::fabs(inputs.turnAbsolutePosition) > 1E-6_rad ) {
        turnRelativeOffset = inputs.turnAbsolutePosition - inputs.turnPosition;
        DataLogger::Log( "Swerve" + m_name + "/turnRelativeOffset", turnRelativeOffset );
    }

    if( angleSetpoint ) {
        // Run closed loop control on angle
        io->setTurnVoltage( m_turnPIDController.Calculate( GetAngle().value(), angleSetpoint->value() ) * 12_V );

        if( speedSetpoint ) {
            units::meters_per_second_t adjustSpeed = speedSetpoint.value() * 
                std::pow( cos( m_turnPIDController.GetError() ), 3); 

            units::radians_per_second_t wheelVelocity = adjustSpeed / swerve::physical::kDriveMetersPerWheelRotation;
            
            io->setDriveVoltage( m_driveFF->Calculate( wheelVelocity ) 
                + m_drivePIDController.Calculate( inputs.driveVelocity.value(), wheelVelocity.value() ) * 12_V );
        }
    }

    // Calculate positions for odometry
    size_t sampleCount = inputs.odometryTimestamps.size();
    odometryPositions.clear();
    for( size_t i=0; i<sampleCount; ++i ) {
        units::meter_t position = inputs.odometryDrivePositions[i] * swerve::physical::kDriveMetersPerWheelRotation;
        units::radian_t angle = inputs.odometryTurnPositions[i] + turnRelativeOffset.value_or(0_rad);
        odometryPositions.push_back( frc::SwerveModulePosition( position, angle ) );
    }
}

// Calculate the angle and speed setpoints
frc::SwerveModuleState Module::RunSetpoint( const frc::SwerveModuleState& state ) {

    // Optimize the state based on the current module turn angle
    frc::SwerveModuleState optimizedState = state;
    optimizedState.Optimize( GetAngle() );

    // Update the setpoints
    angleSetpoint = optimizedState.angle.Radians();
    speedSetpoint = optimizedState.speed;

     return optimizedState;
}

void Module::RunCharacterization( const units::volt_t volts ) {
        // Keep wheels straight
    angleSetpoint = 0_deg;

        // Turn off closed loop velocity control
    speedSetpoint.reset();
    io->setDriveVoltage( volts );
}

units::radian_t Module::GetAngle() {
    if( turnRelativeOffset ) {
        return inputs.turnPosition + turnRelativeOffset.value();
    } else {
        return 0_rad;
    }
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState Module::GetState( void ) {
    return { inputs.driveVelocity * swerve::physical::kDriveMetersPerWheelRotation, GetAngle() };
}

frc::SwerveModulePosition Module::GetPosition( void ) {
    return { inputs.drivePosition * swerve::physical::kDriveMetersPerWheelRotation, GetAngle() };
}

void Module::Stop() {
    io->setTurnVoltage( 0_V );
    io->setDriveVoltage( 0_V );

    angleSetpoint.reset();
    speedSetpoint.reset();
}

void Module::UpdateInputs() {
    io->UpdateInputs( inputs );
}

void ModuleIO::Inputs::processInputs( std::string key ) {
    AUTOLOG( key, drivePosition )
    AUTOLOG( key, driveVelocity )
    AUTOLOG( key, driveAppliedVolts )
    AUTOLOG( key, driveCurrent )

    AUTOLOG( key, turnAbsolutePosition )
    AUTOLOG( key, turnPosition )
    AUTOLOG( key, turnVelocity )
    AUTOLOG( key, turnAppliedVolts )
    AUTOLOG( key, turnCurrent )

    AUTOLOG( key, odometryTimestamps )
    AUTOLOG( key, odometryDrivePositions )
    AUTOLOG( key, odometryTurnPositions )
}
