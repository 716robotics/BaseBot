// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#define ROBOTDISTANCEPERPULSE 0.05333333
#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Talon.h>
#include <frc/Relay.h>
#include <frc/Servo.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include <frc/Compressor.h>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
class Robot : public frc::TimedRobot {
  //Input Devices:
  frc::Joystick leftDriveStick{0};
  frc::Joystick rightDriveStick{1};
  frc::XboxController gamepad{2};
  //Drive motors
  frc::Talon lDrive0{0};
  frc::Talon lDrive1{1};
  frc::Talon rDrive0{2};
  frc::Talon rDrive1{3};
  frc::SpeedControllerGroup lDrive{lDrive0, lDrive1};
  frc::SpeedControllerGroup rDrive{rDrive0, rDrive1};
  frc::DifferentialDrive drive{lDrive, rDrive};
  //Effectors
  frc::Compressor compressor;
	frc::Encoder leftDriveEncoder{0,1,false,frc::Encoder::k4X};
	frc::Encoder rightDriveEncoder{2,3,false,frc::Encoder::k4X};
  //Global Vars
  bool sdfr = false;
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
  void StraightDrive();
  void Abort();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
