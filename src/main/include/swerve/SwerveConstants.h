#pragma once

#include <numbers>
#include <units/math.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include "swerve/ModuleIO.h"

const TuningParams turnTune = { .1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };      // turn PIDSGVA
const TuningParams driveTune = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0013, 0.0 };   // drive PIDSGVA

const char canivoreCanBus[] = "Drivetrain";

const int pigeon2Id = 13;

const ModuleConfigs flconfig = {
    1,          // index
    canivoreCanBus,    // canBus
    1,          // turnCanId
    2,          // driveCanId
    9,          // encoderCanId
    -0.322,     // absoluteEncoderOffset
    turnTune,   // turn PIDSGVA
    driveTune   // drive PIDSGVA
};

const ModuleConfigs frconfig = {
    2,          // index
    canivoreCanBus,    // canBus
    3,          // turnCanId
    4,          // driveCanId
    10,         // encoderCanId
    0.342,      // absoluteEncoderOffset
    turnTune,   // turn PIDSGVA
    driveTune   // drive PIDSGVA
};

const ModuleConfigs blconfig = {
    3,          // index
    canivoreCanBus,    // canBus
    5,          // turnCanId
    6,          // driveCanId
    11,         // encoderCanId
    0.097,      // absoluteEncoderOffset
    turnTune,   // turn PIDSGVA
    driveTune   // drive PIDSGVA
};

const ModuleConfigs brconfig = {
    4,          // index
    canivoreCanBus,    // canBus
    7,          // turnCanId
    8,          // driveCanId
    12,         // encoderCanId
    0.390,      // absoluteEncoderOffset
    turnTune,   // turn PIDSGVA
    driveTune   // drive PIDSGVA
};

namespace swerve {
    namespace pidf {
            // Holonomic Controller Constants
        constexpr double X_Holo_kP = 1;
        constexpr double X_Holo_kI = 0;
        constexpr double X_Holo_kD = 0;

        constexpr double Y_Holo_kP = 1;
        constexpr double Y_Holo_kI = 0;
        constexpr double Y_Holo_kD = 0;

        constexpr double Th_Holo_kP = 1;
        constexpr double Th_Holo_kI = 0;
        constexpr double Th_Holo_kD = 0;

        constexpr units::radians_per_second_t Th_Holo_MaxVel = 6.28_rad_per_s;
        constexpr units::radians_per_second_squared_t Th_Holo_MaxAcc = 3.14_rad_per_s_sq;
    }

    namespace deviceIDs {
        constexpr int kFrontLeftTurnMotorID = 1;
        constexpr int kFrontLeftDriveMotorID = 2;
        constexpr int kFrontRightTurnMotorID = 3;
        constexpr int kFrontRightDriveMotorID = 4;
        constexpr int kBackLeftTurnMotorID = 5;
        constexpr int kBackLeftDriveMotorID = 6;
        constexpr int kBackRightTurnMotorID = 7;
        constexpr int kBackRightDriveMotorID = 8;

        constexpr int kFrontLeftAbsoluteEncoderID = 9;
        constexpr int kFrontRightAbsoluteEncoderID = 10;
        constexpr int kBackLeftAbsoluteEncoderID = 11;
        constexpr int kBackRightAbsoluteEncoderID = 12;

        const int kPigeonIMUID = 13;
    }

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


        constexpr double kFrontLeftAbsoluteOffset = -0.322;
        constexpr double kFrontRightAbsoluteOffset = 0.342;
        constexpr double kBackLeftAbsoluteOffset = 0.097;
        constexpr double kBackRightAbsoluteOffset = 0.39;
    }
}