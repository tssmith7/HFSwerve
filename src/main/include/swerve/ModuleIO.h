#pragma once

#include <string>
#include <vector>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>

#include <frc/geometry/Rotation2d.h>

/**
 * Controller Parameters kP, kI, kD, kS, kG, kV, kA
*/
struct TuningParams {
    double kP;
    double kI;
    double kD;
    double kS;
    double kG;
    double kV;
    double kA;
};

template<class Distance> struct MotionParams {
    using Distance_t = units::unit_t<Distance>;
    using Velocity =
        units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<Velocity>;
    using Acceleration =
        units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using Acceleration_t = units::unit_t<Acceleration>;
    using Jerk =
        units::compound_unit<Acceleration, units::inverse<units::seconds>>;
    using Jerk_t = units::unit_t<Jerk>;

    Velocity_t MaxVelocity;
    Acceleration_t MaxAcceleration;
    Jerk_t MaxJerk;
};

template<class Distance> struct MotionConfig {
    TuningParams tuner;
    MotionParams<Distance> mp;
};

struct ModuleConfigs {
    int index;
    std::string canBus = "";
    int turnCanId;
    int driveCanId;
    int encoderCanId;
    units::turn_t absoluteEncoderOffset;
    TuningParams turnTune;
    TuningParams driveTune;
};


class ModuleIO {
public:
    struct Inputs {
        units::radian_t drivePosition = 0_rad;
        units::radians_per_second_t driveVelocity = 0_rad_per_s;
        units::volt_t driveAppliedVolts = 0_V;
        units::ampere_t driveCurrent = 0_A;

        units::radian_t turnAbsolutePosition = 0_rad;
        units::radian_t turnPosition = 0_rad;
        units::radians_per_second_t turnVelocity = 0_rad_per_s;
        units::volt_t turnAppliedVolts = 0_V;
        units::ampere_t turnCurrent = 0_A;

        std::vector<units::second_t> odometryTimestamps;
        std::vector<units::radian_t> odometryDrivePositions;
        std::vector<units::radian_t> odometryTurnPositions;

        void processInputs( std::string key );
    };

    virtual void UpdateInputs(Inputs &inputs) =0;

    virtual void setDriveVoltage( units::volt_t volts ) =0;

    virtual void setTurnVoltage( units::volt_t volts ) =0;

    virtual void setDriveBrakeMode( bool enable ) =0;

    virtual void setTurnBrakeMode( bool enable ) =0;
};