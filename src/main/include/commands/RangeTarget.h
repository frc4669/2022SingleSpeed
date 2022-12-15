// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

#include "Constants.h"
#include "subsystems/Vision.h"
#include "subsystems/Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RangeTarget
    : public frc2::CommandHelper<frc2::CommandBase, RangeTarget> {
 public:
  RangeTarget(Drivetrain* drivetrain, Vision* vision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivetrain* m_drivetrain;
  Vision* m_vision;

  frc::PIDController m_controller { 0.2, 0, 0 };
};