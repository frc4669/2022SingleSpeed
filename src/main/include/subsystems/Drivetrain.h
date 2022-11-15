// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "Constants.h"
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void ResetEncoders();

  void ConfigureMotor(WPI_TalonFX &motor, bool isInverted);

  void CurvatureDrive(double forward, double rotation); 

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // motors 
  WPI_TalonFX m_leftMain{ DriveConstants::kLeftMain };
  WPI_TalonFX m_leftSecondary{ DriveConstants::kLeftSecondary };
  WPI_TalonFX m_rightMain{ DriveConstants::kRightMain };
  WPI_TalonFX m_rightSecondary{ DriveConstants::kRightSecondary };

  // Link motor controllers together (since we have two on each gearbox)
  frc::MotorControllerGroup m_leftMotors{ m_leftMain, m_leftSecondary };
  frc::MotorControllerGroup m_rightMotors{ m_rightMain, m_rightSecondary };

  // main drive object
  frc::DifferentialDrive m_drive{ m_leftMotors, m_rightMotors };
};
