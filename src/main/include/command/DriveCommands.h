#pragma once

#include <frc2/command/CommandPtr.h>

class Drive;

class DriveCommands {
public:
    static frc2::CommandPtr JoystickDrive( 
        Drive *, 
        std::function<double()> xSupplier, 
        std::function<double()> ySupplier, 
        std::function<double()> omegaSupplier);

private:
    DriveCommands() = default;
};