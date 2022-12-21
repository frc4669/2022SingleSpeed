// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/ExampleAutoRoutine.h"

// DO NOT USE THIS COMMAND IN PRACTICE - it is an example
// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ExampleAutoRoutine::ExampleAutoRoutine(Drivetrain* drivetrain, Vision* vision) {
  AddRequirements({ drivetrain, vision });
  AddCommands(
    FollowTrajectory(drivetrain, "OneMeter", DriveConstants::kMaxAutoSpeed, DriveConstants::kMaxAutoAccel),
    NewGoToTarget(drivetrain, vision)
  );
}
