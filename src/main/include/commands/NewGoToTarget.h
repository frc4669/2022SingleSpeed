// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/Timer.h>

#include <photonlib/PhotonPipelineResult.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Vision.h"

using namespace photonlib;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class NewGoToTarget
    : public frc2::CommandHelper<frc2::CommandBase, NewGoToTarget> {
 public:
  NewGoToTarget(Drivetrain* drivetrain, Vision* vision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
 private:
  
  // subsystems
  Drivetrain* m_drivetrain;
  Vision* m_vision;

  // trajectory generation
  frc::Trajectory m_trajectory;
  frc::TrajectoryConfig m_trajectoryConfig { OperatorConstants::kTeleopAutoVelocity, OperatorConstants::kTeleopAutoAcceleration };

  // feedback and feedforward control, trajectory following
  frc::RamseteController m_ramsete;
  frc::PIDController m_leftPID { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };
  frc::PIDController m_rightPID { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };
  frc::SimpleMotorFeedforward<units::meters> m_feedforward { DriveConstants::ks, DriveConstants::kv, DriveConstants::ka };
  frc::DifferentialDriveKinematics m_kinematics { DriveConstants::kTrackWidth };

  // trajectory timing
  frc::Timer m_timer;
  units::second_t m_previousTime;

  bool m_finished;
};
