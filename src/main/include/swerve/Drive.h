// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/smartdashboard/Field2d.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "swerve/GyroIO.h"
#include "swerve/Module.h"

class Drive : public frc2::SubsystemBase {
public:
    Drive(GyroIO*, ModuleIO*, ModuleIO*, ModuleIO*, ModuleIO*);
    
    // void ArcadeDrive( double xPercent, double yPercent, double omegaPercent );
    void RunVelocity( frc::ChassisSpeeds speeds );
    void Periodic();

    void Stop();

    frc::Pose2d GetPose();
    void SetPose( frc::Pose2d pose );
    frc::Rotation2d GetRotation();

    void ResetDriverOrientation( units::degree_t angle );

    wpi::array<frc::SwerveModuleState,4U>& GetModuleStates();

    frc2::CommandPtr SysIdQuasistatic( frc2::sysid::Direction dir ) { return sysId->Quasistatic( dir ); }
    frc2::CommandPtr SysIdDynamic( frc2::sysid::Direction dir ) { return sysId->Dynamic( dir ); }

private:
    std::unique_ptr<Module> m_modules[4];
    std::unique_ptr<GyroIO> m_gyro;

    GyroIO::Inputs gyroInputs;
    units::radian_t rawGyroRotation{0_rad};
    wpi::array<frc::SwerveModulePosition,4> lastModulePositions{wpi::empty_array};

    frc::SwerveDriveKinematics<4> m_kinematics;
    frc::SwerveDrivePoseEstimator<4> m_odometry;

    std::unique_ptr<frc2::sysid::SysIdRoutine> sysId;

    units::degree_t field_offset;
    units::degree_t driver_offset;
    bool m_have_driver_offset{ false  };

public:
    frc::Field2d m_field;
};
