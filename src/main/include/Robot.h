// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/TalonSRXSimCollection.h>
#include <ctre/phoenix/motorcontrol/TalonFXSimCollection.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/Timer.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Normal";
  const std::string kAutoNameInverted = "Inverted";
  std::string m_autoSelected;
  
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_SRX{0};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_FX{0};

  frc::PWMSparkMax m_spark{0};
  frc::Encoder m_sparkEncoder{0, 1, true};
  frc::sim::EncoderSim m_sparkEncoderSim{m_sparkEncoder};

  ctre::phoenix::motorcontrol::TalonSRXSimCollection m_SRXSim{m_SRX};
  ctre::phoenix::motorcontrol::TalonFXSimCollection m_FXSim{m_FX};


  frc::sim::DifferentialDrivetrainSim m_driveSimSRX =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
  frc::sim::DifferentialDrivetrainSim m_driveSimFX =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
  frc::sim::DifferentialDrivetrainSim m_driveSimSpark =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
};
