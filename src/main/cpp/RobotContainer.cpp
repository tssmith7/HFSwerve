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
    new GyroIOPigeon2( pigeon2Id, flconfig.canBus ), 
    new ModuleIOTalonFX( flconfig ),
    new ModuleIOTalonFX( frconfig ),
    new ModuleIOTalonFX( blconfig ),
    new ModuleIOTalonFX( brconfig )
    );
  } else {
    m_drive = new Drive( 
      new GyroIO(), 
      new ModuleIOSim( ),
      new ModuleIOSim( ),
      new ModuleIOSim( ),
      new ModuleIOSim( )
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


    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (m_controller.Back() && m_controller.Y()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kForward));
    (m_controller.Back() && m_controller.X()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kReverse));
    (m_controller.Start() && m_controller.Y()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (m_controller.Start() && m_controller.X()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
