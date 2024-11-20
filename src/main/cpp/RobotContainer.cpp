// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

#include "swerve/GyroIOPigeon2.h"
#include "swerve/ModuleIOTalonFX.h"

#include "swerve/SwerveConstants.h"

RobotContainer::RobotContainer() {

  m_drive = new Drive( 
    new GyroIOPigeon2( 22, "drive" ), 
    new ModuleIOTalonFX( flconfig ),
    new ModuleIOTalonFX( frconfig ),
    new ModuleIOTalonFX( blconfig ),
    new ModuleIOTalonFX( brconfig )
  );

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
