// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain() {
  // Disable safety on the drivetrain motors
  m_drive.SetSafetyEnabled(false);

  // Configure the drivetrain motors (for now)
  ConfigureMotor(m_leftMain, true);
  ConfigureMotor(m_leftSecondary, true);
  m_leftSecondary.Follow(m_leftMain); // set back left motor to follow the front left motor

  ConfigureMotor(m_rightMain, false);
  ConfigureMotor(m_rightSecondary, false);
  m_rightSecondary.Follow(m_rightMain); // set back right motor to follow the front right motor

  // Reset encoder values to 0 (this also syncs the motor controllers)
  ResetEncoders();

  frc::SmartDashboard::PutData("Field", &m_field);
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {
  m_odometry.Update(GetRotation(), -GetLeftDistance(), -GetRightDistance());

  m_field.SetRobotPose(m_odometry.GetPose());

  frc::SmartDashboard::PutNumber("Left Distance", -GetLeftDistance().value());
  frc::SmartDashboard::PutNumber("Right Distance", -GetRightDistance().value());

  frc::SmartDashboard::PutNumber("Left Velocity", GetLeftVelocity().value());
  frc::SmartDashboard::PutNumber("Right Velocity", GetRightVelocity().value());
}

void Drivetrain::ResetEncoders() {
  // Reset left
  m_leftMain.GetSensorCollection().SetIntegratedSensorPosition(0);
  // Reset Right
  m_rightMain.GetSensorCollection().SetIntegratedSensorPosition(0);
}

void Drivetrain::CurvatureDrive(double speed, double rotation) {
  m_drive.CurvatureDrive(speed, rotation, OperatorConstants::kCanTurnInPlace); 
}

void Drivetrain::ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation) {
  ResetEncoders();
  m_imu.Reset();
  m_odometry.ResetPosition(pose, rotation);
}

void Drivetrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMain.SetVoltage(left);
  m_rightMain.SetVoltage(right);
}

frc::Pose2d Drivetrain::GetOdometryPose() {
  return m_odometry.GetPose();
}

frc::Rotation2d Drivetrain::GetRotation() {
  return frc::Rotation2d(m_imu.GetAngle());
}

units::meters_per_second_t Drivetrain::GetLeftVelocity() {
  double ticksPerSecond = m_leftMain.GetSensorCollection().GetIntegratedSensorVelocity() * 10;

  return -units::meters_per_second_t(
    units::meter_t(units::inch_t(ticksPerSecond * DriveConstants::kInchesPerTick)).value()
  );
}

units::meters_per_second_t Drivetrain::GetRightVelocity() {
  double ticksPerSecond = m_rightMain.GetSensorCollection().GetIntegratedSensorVelocity() * 10;

  return units::meters_per_second_t(
    units::meter_t(units::inch_t(ticksPerSecond * DriveConstants::kInchesPerTick)).value()
  );
}

units::meter_t Drivetrain::GetLeftDistance() {
  return units::meter_t(
    units::inch_t(
      m_leftMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTick
    )
  );
}

units::meter_t Drivetrain::GetRightDistance() {
  return -units::meter_t(
    units::inch_t(
      m_rightMain.GetSensorCollection().GetIntegratedSensorPosition() * DriveConstants::kInchesPerTick
    )
  );
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
  return { GetLeftVelocity(), GetRightVelocity() };
}

frc::Field2d* Drivetrain::GetField() {
  return &m_field;
}

void Drivetrain::ConfigureMotor(WPI_TalonFX &motor, bool isInverted) {
  // set the max velocity and acceleration for motion magic
  motor.ConfigMotionCruiseVelocity(20000);
  motor.ConfigMotionAcceleration(7000);

  // set the current limit for the supply/output current
  motor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 25, 0.5));
  motor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 25, 25, 0.5));

  // time it takes for the motor to go from 0 to full power (in seconds) in an open/closed loop
  motor.ConfigOpenloopRamp(0.2);
  motor.ConfigClosedloopRamp(0);

  // when controller is neutral, set motor to break
  motor.SetNeutralMode(NeutralMode::Brake);

  // disable motor safety
  motor.SetSafetyEnabled(false);

  // motor set experation time
  motor.SetExpiration(100_ms);

  motor.SetInverted(isInverted);

  // Motor PID values (for now)
  motor.Config_kP(0, 0.01); 
  motor.Config_kD(0, 0.00); 
  motor.Config_kF(0, 0.00); 
}