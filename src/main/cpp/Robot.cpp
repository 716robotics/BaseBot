// FRC Team 716 Basic Drive code
// not reccomended for general use
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoDriveForward, kAutoDriveForward);
  m_chooser.AddOption(kAutoDoNothing, kAutoDoNothing);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  compressor.Start();
}

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  leftDriveEncoder.Reset();
  rightDriveEncoder.Reset();
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoDriveForward && autoactive) {
    if (DistanceDrive(1,AUTODIST, true) == DONE) autoactive = false;
  }
  else drive.TankDrive(0,0,false); 
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
  //Analog Controls
  if (gamepad.GetTriggerAxis(gamepad.kRightHand) > 0.1 || gamepad.GetTriggerAxis(gamepad.kLeftHand) > 0.1){ // check deadzone
    auxSpeedController1.Set(gamepad.GetTriggerAxis(gamepad.kRightHand)-gamepad.GetTriggerAxis(gamepad.kLeftHand));} //left is reverse
  if (fabs(gamepad.GetY(gamepad.kLeftHand)) > 0.1 ){ // check deadzone
    auxSpeedController2.Set(gamepad.GetY(gamepad.kLeftHand));} 
  if (fabs(gamepad.GetY(gamepad.kRightHand)) > 0.1 ){ // check deadzone
    auxSpeedController3.Set(gamepad.GetY(gamepad.kRightHand));} 
  //Relay
  if (gamepad.GetBumper(gamepad.kLeftHand)) relay1.Set(relay1.kOn);
  else relay1.Set(relay1.kOff);
  if (gamepad.GetBumper(gamepad.kRightHand)) relay2.Set(relay2.kOn);
  else relay2.Set(relay2.kOff);
  if (gamepad.GetPOV() != -1) relay3.Set(relay3.kOn);
  else relay3.Set(relay3.kOff);

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
  auxSpeedController1.StopMotor();
  auxSpeedController2.StopMotor();
  auxSpeedController3.StopMotor();
  relay1.StopMotor();
  relay2.StopMotor();
  relay3.StopMotor();
}

int Robot::DistanceDrive (float speed, float distance, bool brake)
{
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
	static float autoStartSpeed;
  static float direction;
	static double lastDistance, speedUpDistance, slowDownDistance;
  static int sameCounter;
  static bool brakingFlag;
  static double brakeStartTime; 

	float curve;
	float correction;
	float newSpeed;
	double curDistance;

  if (FirstCallFlag) {
    // Setup distance drive on first call
    // Set initial values for static variables
    brakingFlag = false;
    FirstCallFlag = false;
    if (speed < 0) {
      direction = -1;
    } else {
      direction = 1;
    }
    autoStartSpeed = direction * AUTOSTARTSPEED;
    if (distance < (DRIVERAMPUPDISTANCE * 2)) {
	    speedUpDistance = distance / 2;
	    slowDownDistance = speedUpDistance;
    } else {
	    speedUpDistance = DRIVERAMPUPDISTANCE;
     	slowDownDistance = distance - DRIVERAMPUPDISTANCE;
    }
	  frc::SmartDashboard::PutNumber(  "DistanceDrive Distance", distance);
  	lastDistance = 0;
    sameCounter = 0;
    leftDriveEncoder.Reset();
  }

 	if (brakingFlag) {
     // Braking flag gets set once we reach targe distance if the brake parameter
     // was specified. Drive in reverse direction at low speed for short duration.
    if ((AutoTimer.Get() - brakeStartTime) < .2) {
    	drive.TankDrive(-0.2 * direction *FORWARD, -0.2 * direction * FORWARD);
      return NOTDONEYET;
    } else {
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
	}

  curve = 0;
  
	curDistance = abs(leftDriveEncoder.GetDistance());

	if (curDistance == lastDistance) {
		if (sameCounter++ == 50) {
				return ERROR;
		}
	} else {
		sameCounter = 0;
		lastDistance = curDistance;
	}

	if (curDistance < speedUpDistance) {
		newSpeed = autoStartSpeed + ((speed - autoStartSpeed) * curDistance)/DRIVERAMPUPDISTANCE;
	} else if ((curDistance > slowDownDistance) && (brake == true)) {
		newSpeed = speed * (distance-curDistance)/DRIVERAMPUPDISTANCE;
	} else {
		newSpeed = speed;
	}

	drive.CurvatureDrive(newSpeed * (AUTOFORWARD), (direction * (curve+correction)), false);
//	drive.CurvatureDrive(newSpeed * (AUTOFORWARD), 0, false);
	curDistance = abs(leftDriveEncoder.GetDistance());
  if (curDistance < distance) {
    return NOTDONEYET;
  } else {
    if (brake) {
      brakingFlag = true;
      brakeStartTime = AutoTimer.Get();
      return NOTDONEYET;
    } else {
      FirstCallFlag = true;
      drive.TankDrive(0, 0);
      return DONE;
    }
  }
  
  // should never get here
  drive.TankDrive(0, 0);
  FirstCallFlag = true;
  return DONE;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
