#pragma once

#include <string>
#include <vector>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include "swerve/ModuleIO.h"

// Class for each swerve module on the robot
class Module {
public:
    Module( ModuleIO* io );
    void Periodic();
    frc::SwerveModuleState RunSetpoint( const frc::SwerveModuleState& state );
    void RunCharacterization( const units::volt_t volts );
    void Stop();
    void SetBrakeMode( bool enabled );
    void UpdateInputs();

    units::radian_t GetAngle( void );
    frc::SwerveModuleState GetState( void );
    frc::SwerveModulePosition GetPosition ( void );

    const std::vector<units::second_t>& getOdometryTimestamps() { return inputs.odometryTimestamps; }
    const std::vector<frc::SwerveModulePosition>& getOdometryPositions() { return odometryPositions; }

private:
    std::string m_name;

    std::unique_ptr<ModuleIO> io;

    ModuleIO::Inputs inputs;

    std::optional<units::radian_t> turnRelativeOffset;
    std::optional<units::radian_t> angleSetpoint;
    std::optional<units::meters_per_second_t> speedSetpoint;

    std::vector<frc::SwerveModulePosition> odometryPositions;
};