// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>
#include <wpi/numbers>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameInverted, kAutoNameInverted);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // This is not about motor inversion but about encoder phases.
  // If positive inversion = negative encoder output AND negative inversion = positive encoder output
  // Then flip this.
  // SetInverted() will flip sensor direction as well,
  m_SRX.SetSensorPhase(false);
  m_FX.SetSensorPhase(false);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Spark Get", m_spark.Get());
  frc::SmartDashboard::PutNumber("SRX Get", m_SRX.Get());
  frc::SmartDashboard::PutNumber("FX Get", m_FX.Get());
  frc::SmartDashboard::PutNumber("SRX Vel", m_SRX.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("FX Vel", m_FX.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Spark Vel", m_sparkEncoder.GetRate());
  frc::SmartDashboard::PutNumber("SRX Pos", m_SRX.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("FX Pos", m_FX.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Spark Pos", m_sparkEncoder.GetDistance());

  m_autoSelected = m_chooser.GetSelected();
  if (m_autoSelected == kAutoNameInverted) {
      m_spark.SetInverted(true);
    m_sparkEncoder.SetReverseDirection(true);

    m_SRX.SetInverted(true);

    m_FX.SetInverted(true);
  } else {
    m_spark.SetInverted(false);
    m_sparkEncoder.SetReverseDirection(false);

    m_SRX.SetInverted(false);

    m_FX.SetInverted(false);
  }
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_spark.Set(1);
  m_SRX.Set(1);
  m_FX.Set(1);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  m_driveSimSRX.SetInputs(m_SRX.Get() * 12_V, m_SRX.Get() * 12_V);
  m_driveSimFX.SetInputs(m_FX.Get() * 12_V, m_FX.Get() * 12_V);
  m_driveSimSpark.SetInputs(m_spark.Get() * 12_V, m_spark.Get() * 12_V);

  m_driveSimSRX.Update(20_ms);
  m_driveSimFX.Update(20_ms);
  m_driveSimSpark.Update(20_ms);

  m_SRXSim.SetQuadratureRawPosition((m_SRX.GetInverted() ? -1 : 1) * m_driveSimSRX.GetLeftPosition().value());
  m_SRXSim.SetQuadratureVelocity((m_SRX.GetInverted() ? -1 : 1) * m_driveSimSRX.GetLeftVelocity().value());

  m_FXSim.SetIntegratedSensorRawPosition((m_FX.GetInverted() ? -1 : 1) * m_driveSimFX.GetLeftPosition().value());
  m_FXSim.SetIntegratedSensorVelocity((m_FX.GetInverted() ? -1 : 1) * m_driveSimFX.GetLeftVelocity().value());

  m_sparkEncoderSim.SetDistance(m_driveSimSpark.GetLeftPosition().value());
  m_sparkEncoderSim.SetRate(m_driveSimSpark.GetLeftVelocity().value());
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
