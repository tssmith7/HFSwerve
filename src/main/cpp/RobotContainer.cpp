// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include "swerve/GyroIOPigeon2.h"
#include "swerve/ModuleIOTalonFX.h"
#include "swerve/ModuleIOSim.h"

#include "swerve/SwerveConstants.h"

#include "command/DriveCommands.h"

RobotContainer::RobotContainer() {

  if( frc::RobotBase::IsReal() ) {
    m_drive = new Drive( 
      new GyroIOPigeon2( 22, "drive" ), 
      new ModuleIOTalonFX( swerve::pidf::flconfig ),
      new ModuleIOTalonFX( swerve::pidf::frconfig ),
      new ModuleIOTalonFX( swerve::pidf::blconfig ),
      new ModuleIOTalonFX( swerve::pidf::brconfig )
    );
  } else {
    m_drive = new Drive( 
      new GyroIO(), 
      new ModuleIOSim( swerve::pidf::flconfig ),
      new ModuleIOSim( swerve::pidf::frconfig ),
      new ModuleIOSim( swerve::pidf::blconfig ),
      new ModuleIOSim( swerve::pidf::brconfig )
    );
  }
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_drive->SetDefaultCommand( 
    DriveCommands::JoystickDrive( 
      m_drive,
      [this] { return -m_controller.GetLeftY(); },
      [this] { return -m_controller.GetLeftX(); },
      [this] { return m_controller.GetRightX(); }
    )
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
