#pragma once

#include <queue>

#include <frc/simulation/DCMotorSim.h>

#include "swerve/ModuleIO.h"


class ModuleIOSim : public ModuleIO {
public:
    ModuleIOSim();

    virtual void UpdateInputs(ModuleIO::Inputs& inputs) override;

    virtual void setDriveVoltage( units::volt_t volts ) override;

    virtual void setTurnVoltage( units::volt_t volts ) override;

    virtual void setDriveBrakeMode( bool enable ) override {}

    virtual void setTurnBrakeMode( bool enable ) override {}
    
private:
    frc::sim::DCMotorSim driveSim;
    frc::sim::DCMotorSim turnSim;

    units::radian_t turnAbsoluteInitPosition;
    units::volt_t driveAppliedVolts;
    units::volt_t turnAppliedVolts;
};