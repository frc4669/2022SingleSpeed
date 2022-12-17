// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/RamseteController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/Timer.h>

#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonTrackedTarget.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Vision.h"

class GoToTarget
    : public frc2::CommandHelper<frc2::CommandBase, GoToTarget> {
  public:
    GoToTarget(Drivetrain* drivetrain, Vision* vision);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;
    
    bool IsFinished() override;

  private:
    // required subsystems
    Drivetrain* m_drivetrain; 
    Vision* m_vision;

    // linear feedback control (for each side of the drivetrain)
    frc::PIDController m_leftControl { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };
    frc::PIDController m_rightControl { DriveConstants::kp, DriveConstants::ki, DriveConstants::kd };

    // nonlinear feedback control (for the position of the entire robot)
    frc::RamseteController m_ramsete;
    frc::DifferentialDriveKinematics m_kinematics { DriveConstants::kTrackWidth };

    // feedforward control (to account for known imperfections in the drivetrain)
    frc::SimpleMotorFeedforward<units::meters> m_feedforward { DriveConstants::ks, DriveConstants::kv, DriveConstants::ka };

    frc::Timer m_timer;
    units::second_t m_previousTime;
    frc::DifferentialDriveWheelSpeeds m_previousSpeed;

    units::meters_per_second_t m_desiredVelocity;
};
