#pragma once

#include <queue>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "swerve/GyroIO.h"

class GyroIOPigeon2 : public GyroIO {
public:
    GyroIOPigeon2( int canId, std::string canBus );
    virtual void UpdateInputs(Inputs &inputs);

private:
    ctre::phoenix6::hardware::Pigeon2 pigeon;

    ctre::phoenix6::StatusSignal<units::degree_t> yaw = pigeon.GetYaw();
    ctre::phoenix6::StatusSignal<units::degrees_per_second_t> yawVelocity = pigeon.GetAngularVelocityZWorld();
    std::queue<units::second_t>* yawTimestampQueue;
    std::queue<double>* yawPositionQueue;
};