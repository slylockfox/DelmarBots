/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Joystick.h>
#include <frc/Talon.h>
#include <frc/Victor.h>
#include <frc/Spark.h>
#include <frc/Timer.h>
#include <frc/Servo.h>
#include <frc/DigitalInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  Robot() {    
      m_robotDrive.SetExpiration(0.1);    
      m_timer.Start();  
    }

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  const double kSlowSpeedFactor = 1.7;
  std::string m_autoSelected;
  std::shared_ptr<NetworkTable> m_limetable;  // for LimeLight

  // Robot drive system  
  frc::Talon m_left{0};  
  frc::Talon m_right{1}; 

  frc::Victor m_chainsaw{7};
  frc::Spark m_chainsaw_dart{6};

  frc::Spark m_lift{3};

  frc::DigitalInput m_bottom_switch{9};
  frc::DigitalInput m_top_switch{8};

  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  frc::Joystick m_stick{0};  
  frc::Joystick m_stick_copilot{1};  
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();  
  frc::Timer m_timer;
  frc::Servo m_limeServo{2};
  double m_limeServoAngle = 90.0;
  bool m_okToPursue = false;
  double speed_factor = kSlowSpeedFactor;





};
