#pragma once

#include <queue>

#include "swerve/ModuleIO.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>


class ModuleIOTalonFX : public ModuleIO {
public:
    ModuleIOTalonFX( ModuleConfigs configs );

    virtual void UpdateInputs(ModuleIO::Inputs& inputs) override;

    virtual void setDriveVoltage( units::volt_t volts ) override;

    virtual void setTurnVoltage( units::volt_t volts ) override;

    virtual void setDriveBrakeMode( bool enable ) override;

    virtual void setTurnBrakeMode( bool enable ) override;

private:
    ctre::phoenix6::hardware::TalonFX m_driveMotor;
    ctre::phoenix6::hardware::TalonFX m_turnMotor;
    ctre::phoenix6::hardware::CANcoder m_encoder;

    std::queue<units::second_t>* timestampQueue;

    ctre::phoenix6::StatusSignal<units::turn_t> drivePosition = m_driveMotor.GetPosition();
    std::queue<double>* drivePositionQueue;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocity = m_driveMotor.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> driveAppliedVolts = m_driveMotor.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> driveCurrent = m_driveMotor.GetSupplyCurrent();

    ctre::phoenix6::StatusSignal<units::turn_t> turnAbsolutePosition = m_encoder.GetAbsolutePosition();
    ctre::phoenix6::StatusSignal<units::turn_t> turnPosition = m_turnMotor.GetPosition();
    std::queue<double>* turnPositionQueue;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> turnVelocity = m_turnMotor.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> turnAppliedVolts = m_turnMotor.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> turnCurrent = m_turnMotor.GetSupplyCurrent();

        // SDS Mk4i L2 ratios
    const double driveGearRatio = 6.75;
    const double turnGearRatio = 150.0 / 7.0;

    bool isTurnMotorInverted = true;
};