#pragma once

#include <vector>

#include <units/angle.h>
#include <units/angular_velocity.h>

class GyroIO {
public:
    struct Inputs {
        bool connected = false;
        units::radian_t yawPosition = 0_rad;
        units::radians_per_second_t yawVelocity = 0_rad_per_s;

        std::vector<units::second_t> odometryYawTimestamps;
        std::vector<units::radian_t> odometryYawPositions;
        std::vector<units::radian_t> odometryTurnPositions;

        void processInputs( std::string key );
    };

    virtual void UpdateInputs(Inputs &inputs) =0;
};