// FRC Team 716 Basic Drive code
// not reccomended for general use
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  compressor.Start();
}

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

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

void Robot::TeleopInit() {
  leftDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
}

void Robot::TeleopPeriodic() {
  if (leftDriveStick.GetTrigger()) StraightDrive();
  else {drive.TankDrive((leftDriveStick.GetY() * -1), (rightDriveStick.GetY() * -1));
  sdfr = false;}
  if (gamepad.GetBackButtonPressed()) Abort();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::StraightDrive(){
//drive.TankDrive
if (!sdfr){
  leftDriveEncoder.Reset();
  rightDriveEncoder.Reset();
  sdfr = true;
}
double throttle = (-1 *leftDriveStick.GetY());
double left = (leftDriveEncoder.GetDistance());
double right = (-1 * rightDriveEncoder.GetDistance());
double difference = right - left;
double lpower = (throttle - (difference * 0.1));
double rpower = (throttle + (difference * 0.1));
drive.TankDrive(lpower, rpower, false);
}

void Robot::Abort(){
  return;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
