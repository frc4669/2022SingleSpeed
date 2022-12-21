// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "commands/FollowTrajectory.h"
#include "commands/NewGoToTarget.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Vision.h"

class ExampleAutoRoutine
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ExampleAutoRoutine> {
 public:
  ExampleAutoRoutine(Drivetrain* drivetrain, Vision* vision);
};
