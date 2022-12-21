// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "commands/FollowTrajectory.h"
#include "subsystems/Drivetrain.h"

class DefaultAutoRoutine
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 DefaultAutoRoutine> {
 public:
  DefaultAutoRoutine(Drivetrain* drivetrain);
};
