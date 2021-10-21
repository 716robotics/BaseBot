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
  if (rightDriveStick.GetTrigger()) HoldTheLine();
  else if (leftDriveStick.GetTrigger()) StraightDrive();
  else {
    drive.TankDrive((leftDriveStick.GetY() * -1), (rightDriveStick.GetY() * -1));
    sdfr = false;}
  if (gamepad.GetBackButtonPressed()) Abort();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::StraightDrive(){
  if (!sdfr){
    leftDriveEncoder.Reset();
    rightDriveEncoder.Reset();
    sdfr = true;
  }
  double throttle = (-1 *leftDriveStick.GetY());
  double difference = (-1 * rightDriveEncoder.GetDistance()) - (leftDriveEncoder.GetDistance());
  drive.TankDrive((throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
  }

//Should keep the robot from moving, never tested it
void Robot::HoldTheLine(){
  std::cout << "Love isn't always on time" << std::endl; // No I am not ashamed of this TOTO reference
    if (!sdfr){
    leftDriveEncoder.Reset();
    rightDriveEncoder.Reset();
    sdfr = true;
  }
  double ldrift = leftDriveEncoder.GetDistance();
  drive.TankDrive((0.5 * leftDriveEncoder.GetDistance()),(-0.5 * rightDriveEncoder.GetDistance()), false);
}

void Robot::Abort(){
  return;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
