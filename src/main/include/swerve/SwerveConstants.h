#pragma once

#include <numbers>
#include <units/math.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>

#include "swerve/ModuleIO.h"

    // Create the jerk unit meters_per_second_cubed
namespace units {
UNIT_ADD(jerk, meters_per_second_cubed, meters_per_second_cubed,
         mps_cu, compound_unit<length::meter, inverse<cubed<time::seconds>>>)
}

namespace swerve {

    namespace physical {
        // The width of the drive base from the center of one module to another adjacent one.
        constexpr units::meter_t kDriveBaseWidth = 23.25_in * 1.08;
        constexpr units::meter_t kDriveBaseLength = 22.5_in * 1.08;

        const units::meter_t kDriveBaseRadius = 
            units::math::hypot(swerve::physical::kDriveBaseWidth, swerve::physical::kDriveBaseLength) / 2.0;

        // The wheel diameter.
        constexpr units::inch_t kWheelDiameter = 4_in;

        // The max RPM of the drive motors (Falcon 500s)
        constexpr units::revolutions_per_minute_t kDriveRPM = 6380_rpm;

        // Gear ratio of the drive motors for a SDS Mk4i L2
        // 6.75 rotations of the drive motor is one rotation of the wheel.
        constexpr double kDriveGearRatio = 6.75;

        // Compound unit for the meter per revolution constant.
        using meters_per_rev = units::compound_unit<units::meters, units::inverse<units::turns>>;
        typedef units::unit_t<meters_per_rev> meters_per_rev_t;

        // The number of meters traveled per rotation of the drive motor
        // wheel circumference / gear ratio
        constexpr meters_per_rev_t kDriveMetersPerWheelRotation = std::numbers::pi * kWheelDiameter / 1_tr;

        // Max drive speed of Mk4i swerve modules when motors turn maximum RPM.
        constexpr units::meters_per_second_t kMaxDriveSpeed = kDriveRPM * kDriveMetersPerWheelRotation / kDriveGearRatio;
        const units::radians_per_second_t kMaxTurnSpeed = 1_rad * (kMaxDriveSpeed / kDriveBaseRadius);

        // Gear ratio of the turn motors for SDS Mk4i.
        constexpr double kTurnGearRatio = 150.0 / 7.0;

        // The Maximum turning speed for the robot under Joystick control
        const units::revolutions_per_minute_t kTurnSpeedLimit = kMaxTurnSpeed;

        // The Maximum translation speed for the robot under Joystick control
        constexpr units::meters_per_second_t kDriveSpeedLimit = kMaxDriveSpeed;
    }

    namespace pidf {

        const TuningParams turnTune = { 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };      // turn PIDSGVA
        const TuningParams driveTune = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.113, 0.0 };   // drive PIDSGVA

        const MotionParams<units::radian> turnMP = { 540_deg_per_s, 720_deg_per_s_sq, 0_deg_per_s_cu };
        const MotionParams<units::radian> driveMP = { 
            4.5_mps / physical::kDriveMetersPerWheelRotation, 
            8_mps_sq / physical::kDriveMetersPerWheelRotation, 
            0_mps_cu / physical::kDriveMetersPerWheelRotation 
        };

        const ModuleConfigs flconfig = {
            0,          // index
            "drive",    // canBus
            1,          // driveCanId
            2,          // turnCanId
            9,          // encoderCanId
            -0.322,     // absoluteEncoderOffset
            {turnTune, turnMP},   // turn control
            {driveTune, driveMP}  // drive control
        };

        const ModuleConfigs frconfig = {
            1,          // index
            "drive",    // canBus
            3,          // driveCanId
            4,          // turnCanId
            10,         // encoderCanId
            0.342,      // absoluteEncoderOffset
            {turnTune, turnMP},   // turn control
            {driveTune, driveMP}  // drive control
        };

        const ModuleConfigs blconfig = {
            2,          // index
            "drive",    // canBus
            5,          // driveCanId
            6,          // turnCanId
            11,         // encoderCanId
            0.097,      // absoluteEncoderOffset
            {turnTune, turnMP},   // turn control
            {driveTune, driveMP}  // drive control
        };

        const ModuleConfigs brconfig = {
            3,          // index
            "drive",    // canBus
            7,          // driveCanId
            8,          // turnCanId
            12,         // encoderCanId
            0.390,      // absoluteEncoderOffset
            {turnTune, turnMP},   // turn control
            {driveTune, driveMP}  // drive control
        };
    }
}