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
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_motor0{0};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_motor1{1};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_motor2{2};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_falcon0{0};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_falcon1{1};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX m_falcon2{2};

  ctre::phoenix::motorcontrol::TalonSRXSimCollection m_motorSim0{m_motor0};
  ctre::phoenix::motorcontrol::TalonSRXSimCollection m_motorSim1{m_motor1};
  ctre::phoenix::motorcontrol::TalonSRXSimCollection m_motorSim2{m_motor2};
  ctre::phoenix::motorcontrol::TalonFXSimCollection m_falconSim0{m_falcon0};
  ctre::phoenix::motorcontrol::TalonFXSimCollection m_falconSim1{m_falcon1};
  ctre::phoenix::motorcontrol::TalonFXSimCollection m_falconSim2{m_falcon2};


  frc::sim::DifferentialDrivetrainSim m_driveSim1 =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
  frc::sim::DifferentialDrivetrainSim m_driveSim2 =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
  frc::sim::DifferentialDrivetrainSim m_driveSim3 =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
  frc::sim::DifferentialDrivetrainSim m_driveSim4 =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
  frc::sim::DifferentialDrivetrainSim m_driveSim5 =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
  frc::sim::DifferentialDrivetrainSim m_driveSim6 =
    frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
      frc::sim::DifferentialDrivetrainSim::KitbotMotor::DualCIMPerSide, // 2 CIMs per side.
      frc::sim::DifferentialDrivetrainSim::KitbotGearing::k10p71,       // 10.71:1
      frc::sim::DifferentialDrivetrainSim::KitbotWheelSize::kSixInch    // 6" diameter wheels.
  );
};
