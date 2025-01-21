#include <cstdlib>
#include <ctime>
#include <numbers>

#include <units/moment_of_inertia.h>
#include <frc/Timer.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "swerve/SwerveConstants.h"
#include "swerve/ModuleIOSim.h"

ModuleIOSim::ModuleIOSim() :
    driveSim{ frc::LinearSystemId::DCMotorSystem( frc::DCMotor::NEO(), 0.025_kg_sq_m, swerve::physical::kDriveGearRatio ), frc::DCMotor::NEO()},
    turnSim{ frc::LinearSystemId::DCMotorSystem(frc::DCMotor::NEO(), 0.004_kg_sq_m, swerve::physical::kTurnGearRatio), frc::DCMotor::NEO()}
{
    std::srand(std::time(nullptr));
    double random_number = std::rand() / (1.0 * RAND_MAX );  // [0 - 1.0] range
    turnAbsoluteInitPosition = units::radian_t( random_number * 2.0 * std::numbers::pi );
}

void ModuleIOSim::UpdateInputs(Inputs& inputs) {
    driveSim.Update( 20_ms );
    turnSim.Update( 20_ms );

    inputs.drivePosition = driveSim.GetAngularPosition();
    inputs.driveVelocity = driveSim.GetAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrent = driveSim.GetCurrentDraw();

    inputs.turnAbsolutePosition = turnSim.GetAngularPosition() + turnAbsoluteInitPosition;
    inputs.turnPosition = turnSim.GetAngularPosition();
    inputs.turnVelocity = turnSim.GetAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrent = turnSim.GetCurrentDraw();

    inputs.odometryTimestamps.clear();
    inputs.odometryTimestamps.push_back( frc::Timer::GetFPGATimestamp() );

    inputs.odometryDrivePositions.clear();
    inputs.odometryDrivePositions.push_back(inputs.drivePosition);

    inputs.odometryTurnPositions.clear();
    inputs.odometryTurnPositions.push_back(inputs.turnPosition);
}

void ModuleIOSim::setDriveVoltage( units::volt_t volts ) {
    if( volts > 12_V ) volts = 12_V;
    if( volts < -12_V ) volts = -12_V;
    driveAppliedVolts = volts;
    driveSim.SetInputVoltage( driveAppliedVolts );
}

void ModuleIOSim::setTurnVoltage( units::volt_t volts ) {
    if( volts > 12_V ) volts = 12_V;
    if( volts < -12_V ) volts = -12_V;
    turnAppliedVolts = volts;
    turnSim.SetInputVoltage( turnAppliedVolts );
}
