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
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_motor0.SetInverted(false);
  m_motor0.SetSensorPhase(false);
  m_motor1.SetInverted(true);
  m_motor1.SetSensorPhase(true);
  m_motor2.SetInverted(true);
  m_motor2.SetSensorPhase(false);

  m_falcon0.SetInverted(false);
  m_falcon0.SetSensorPhase(false);
  m_falcon1.SetInverted(true);
  m_falcon1.SetSensorPhase(true);
  m_falcon2.SetInverted(true);
  m_falcon2.SetSensorPhase(false);
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
  frc::SmartDashboard::PutNumber("SRX Normal Vel", m_motor0.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("SRX Inverted Vel", m_motor1.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("SRX No SensorPhase Inverted Vel", m_motor2.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Falcon Normal Vel", m_falcon0.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Falcon Inverted Vel", m_falcon1.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Falcon No SensorPhase Inverted Vel", m_falcon2.GetSelectedSensorVelocity());
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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  // Just run a periodic sine function
  m_motor0.Set(-1);
  m_motor1.Set(-1);
  m_motor2.Set(-1);
  m_falcon0.Set(-1);
  m_falcon1.Set(-1);
  m_falcon2.Set(-1);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  m_driveSim1.SetInputs(m_motor0.Get() * 12_V, m_motor0.Get() * 12_V);
  m_driveSim2.SetInputs(m_motor1.Get() * 12_V, m_motor1.Get() * 12_V);
  m_driveSim3.SetInputs(m_motor2.Get() * 12_V, m_motor2.Get() * 12_V);
  m_driveSim4.SetInputs(m_falcon0.Get() * 12_V, m_falcon0.Get() * 12_V);
  m_driveSim5.SetInputs(m_falcon1.Get() * 12_V, m_falcon1.Get() * 12_V);
  m_driveSim6.SetInputs(m_falcon2.Get() * 12_V, m_falcon2.Get() * 12_V);

  m_driveSim1.Update(20_ms);
  m_driveSim2.Update(20_ms);
  m_driveSim3.Update(20_ms);
  m_driveSim4.Update(20_ms);
  m_driveSim5.Update(20_ms);
  m_driveSim6.Update(20_ms);

  m_motorSim0.SetQuadratureRawPosition(m_driveSim1.GetLeftPosition().value());
  m_motorSim1.SetQuadratureRawPosition(m_driveSim2.GetLeftPosition().value());
  m_motorSim2.SetQuadratureRawPosition(m_driveSim3.GetLeftPosition().value());
  m_motorSim0.SetQuadratureVelocity(m_driveSim1.GetLeftVelocity().value());
  m_motorSim1.SetQuadratureVelocity(m_driveSim2.GetLeftVelocity().value());
  m_motorSim2.SetQuadratureVelocity(m_driveSim3.GetLeftVelocity().value());

  m_falconSim0.SetIntegratedSensorRawPosition(m_driveSim4.GetLeftPosition().value());
  m_falconSim1.SetIntegratedSensorRawPosition(m_driveSim5.GetLeftPosition().value());
  m_falconSim2.SetIntegratedSensorRawPosition(m_driveSim6.GetLeftPosition().value());
  m_falconSim0.SetIntegratedSensorVelocity(m_driveSim4.GetLeftVelocity().value());
  m_falconSim1.SetIntegratedSensorVelocity(m_driveSim5.GetLeftVelocity().value());
  m_falconSim2.SetIntegratedSensorVelocity(m_driveSim6.GetLeftVelocity().value());
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
