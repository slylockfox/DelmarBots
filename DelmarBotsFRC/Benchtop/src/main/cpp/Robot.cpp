/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <algorithm>  // for min and max
#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

void Robot::RobotInit() {
  m_limetable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  m_robotDrive.SetRightSideInverted(true);

  //m_left.SetInverted(true);
  //m_right.SetInverted(true);

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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

  TeleopInit();

#if FALSE
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    m_timer.Reset();
    m_timer.Start();
  }
#endif
}


void Robot::AutonomousPeriodic() {

  TeleopPeriodic();

#if FALSE
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0) {
      m_robotDrive.TankDrive(0.5, 0); // left only
    } else if (m_timer.Get() < 4.0) {
      m_robotDrive.TankDrive(0, 0.5); // right only
    } else if (m_timer.Get() < 6.0) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(0.5, 0.0);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
  }
#endif
}

#define TARGET_TOLERANCE_V 4
#define TARGET_TOLERANCE_H 8
#define LIMELIGHT_ANGLE_DEFAULT 65
#define SPEED_ROTATE 0.3
#define SPEED_PURSUE 0.3
#define MIN_TARGET_AREA_PERCENT 1.0
using namespace std;

void Robot::TeleopPeriodic() {

#ifdef LIMELIGHT

  bool ok_to_pursue_button_presssed = m_stick.GetRawButton(2);
  bool not_ok_to_pursue_button_presssed = m_stick.GetRawButton(3);
  bool limelight_cv_mode_pressed = m_stick_copilot.GetRawButton(2);
  bool limelight_driver_mode_pressed = m_stick_copilot.GetRawButton(3);

  if (ok_to_pursue_button_presssed) {
    m_okToPursue = true;
    frc::SmartDashboard::PutString("DB/String 1", "pursue enabled");
  } else if (not_ok_to_pursue_button_presssed) {
    m_okToPursue = false;
    frc::SmartDashboard::PutString("DB/String 1", "pursue disabled");
  }

  if (limelight_cv_mode_pressed) {
    m_limetable->PutNumber("camMode",0.0); // camera in normal CV mode
    m_limetable->PutNumber("ledMode",0.0); // LED auto
    // m_limetable->PutNumber("stream",0.0);  // secondary camera side-by-side
    frc::SmartDashboard::PutString("DB/String 0", "limelight cv mode");
  } else if (limelight_driver_mode_pressed) {
    m_limetable->PutNumber("camMode",1.0); // camera in driver mode
    m_limetable->PutNumber("ledMode",1.0); // LED off
    // m_limetable->PutNumber("stream",1.0);  // secondary camera in PIP
    frc::SmartDashboard::PutString("DB/String 0", "limelight driver mode");
  }

  double speed_left = 0.0;
  double speed_right = 0.0;

  double targetSeen = m_limetable->GetNumber("tv",0.0);
  double targetArea = m_limetable->GetNumber("ta",0.0);
  if (targetSeen != 0.0 && targetArea > MIN_TARGET_AREA_PERCENT) {  // tv is true if there is a target detected
    double targetOffsetAngle_Horizontal = m_limetable->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = m_limetable->GetNumber("ty",0.0);   
    double targetSkew = m_limetable->GetNumber("ts",0.0);

    // vertical elevation by servo
    if (targetOffsetAngle_Vertical > TARGET_TOLERANCE_V) {
      m_limeServoAngle -= 1;
    } else if (targetOffsetAngle_Vertical < -TARGET_TOLERANCE_V) {
      m_limeServoAngle += 1;
    }
    m_limeServoAngle = std::min(m_limeServoAngle, 180.0);
    m_limeServoAngle = std::max(m_limeServoAngle, 0.0);
    //m_limeServoAngle = 90.0 - targetOffsetAngle_Vertical;
    m_limeServo.SetAngle(m_limeServoAngle);

    // drive toward target
    if (m_okToPursue && targetArea <= 10.0 && targetArea > 5) { // pursue target until it's bigger
      speed_left = SPEED_PURSUE; speed_right = SPEED_PURSUE;
    } else if (m_okToPursue && targetArea > 10.0) { // back up!
      speed_left = -SPEED_PURSUE; speed_right = -SPEED_PURSUE;
    }

    // center on target
    if (targetOffsetAngle_Horizontal > TARGET_TOLERANCE_H) {
      speed_left += SPEED_ROTATE;
      speed_right -= SPEED_ROTATE;
    } else if (targetOffsetAngle_Horizontal < -TARGET_TOLERANCE_H) {
      speed_left -= SPEED_ROTATE;
      speed_right += SPEED_ROTATE;
    }

    // move robot
    m_robotDrive.TankDrive(speed_left, speed_right, false);
    // m_robotDrive.TankDrive(0.2, 0.2, false);

  } else {  // no vision target seen
    //   leave vertical alone, was... m_limeServo.SetAngle(LIMELIGHT_ANGLE_DEFAULT);

#else
  // no Limelight; this is GRC 2019 robot

  // pilot controls
  bool slow_gear_button_pressed = m_stick.GetRawButton(3);
  bool high_gear_button_presssed = m_stick.GetRawButton(2);
  if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
  else if (high_gear_button_presssed) {speed_factor = 1;}
  frc::SmartDashboard::PutNumber ("DB/Drive Speed Factor", speed_factor);

#endif

    // switch states
    bool lift_at_bottom = !m_bottom_switch.Get();
    bool lift_at_top = !m_top_switch.Get();
    frc::SmartDashboard::PutNumber ("DB/Switch Bottom", lift_at_bottom);
    frc::SmartDashboard::PutNumber ("DB/Switch Top", lift_at_top);

    // Drive with arcade style (use right stick)   
    double drive_speed_X = m_stick.GetX()/speed_factor;
    double drive_speed_Y = m_stick.GetY()/speed_factor;
    frc::SmartDashboard::PutNumber ("DB/Drive Speed X", drive_speed_X);
    frc::SmartDashboard::PutNumber ("DB/Drive Speed Y", drive_speed_Y);
    m_robotDrive.ArcadeDrive(-drive_speed_Y, drive_speed_X); // MJS: not so fast

    // co-pilot controls chainsaw
    double chainsaw_speed = 0; //m_stick.GetRawAxis(3);
    if (m_stick_copilot.GetPOV(0) == 0) {chainsaw_speed = -0.3;}
    else if (m_stick_copilot.GetPOV(0) == 180) {chainsaw_speed = 0.3;}
    frc::SmartDashboard::PutNumber ("DB/POV", m_stick.GetPOV(0));
    frc::SmartDashboard::PutNumber ("DB/Chainsaw Speed", chainsaw_speed);
    m_chainsaw.Set(chainsaw_speed);

    // co-pilot controls dart
    double dart_speed_Y = -m_stick_copilot.GetRawAxis(3);
    frc::SmartDashboard::PutNumber ("DB/Dart Speed", dart_speed_Y);
    if (abs(dart_speed_Y) < 0.1){
      dart_speed_Y = 0.0;
    }
    m_chainsaw_dart.Set(dart_speed_Y);

    // co-pilot controls lift
     double lift_speed_Y = m_stick_copilot.GetRawAxis(1);
    frc::SmartDashboard::PutNumber ("DB/Lift Speed", lift_speed_Y);
    if (abs(lift_speed_Y) < 0.1){ // dead zone
      lift_speed_Y = 0.0;
    }
    bool ok_to_drive_lift = false;
    if (lift_speed_Y > 0 && !lift_at_bottom) { // not at bottom, so ok to go down
      ok_to_drive_lift = true;
    } else if (lift_speed_Y < 0 && !lift_at_top) { // not at top, so ok to go up
      ok_to_drive_lift = true;
    }
    if (ok_to_drive_lift) { 
      m_lift.Set(lift_speed_Y); 
    } else {
      m_lift.Set(0); 
    }
    


#ifdef LIMELIGHT
  }
#endif

}

void Robot::TestPeriodic() {}
void Robot::TeleopInit() {
  m_limeServo.SetAngle(LIMELIGHT_ANGLE_DEFAULT);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
