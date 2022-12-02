// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/controller/RamseteController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc2/command/PIDCommand.h>

#include <subsystems/Drivetrain.h>
#include <subsystems/Vision.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class GoToTarget
    : public frc2::CommandHelper<frc2::CommandBase, GoToTarget> {
  public:
    GoToTarget(Drivetrain* drivetrain, Vision* vision);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;
    
    bool IsFinished() override;

  private: 
    // subsystem requirements
    Drivetrain* m_drivetrain; 
    Vision* m_vision; 
    
    // // used to calculate output voltage/velocity
    frc::RamseteController m_controller; 
    frc::DifferentialDriveKinematics m_kinematics; 

    // output
    std::function<void(units::meters_per_second_t, units::meters_per_second_t)> m_output;

    // config 
    units::meters_per_second_t desieredVelocity { 0.5 }; 
};
