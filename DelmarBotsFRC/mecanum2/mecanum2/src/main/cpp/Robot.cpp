/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Talon.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "navx/AHRS.h"
#include <frc/DriverStation.h>  // for reporting errors from navx

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 
 private:
  static constexpr int kFrontLeftChannel = 6;
  static constexpr int kRearLeftChannel = 7;
  static constexpr int kFrontRightChannel = 8;
  static constexpr int kRearRightChannel = 9;

  static constexpr int kJoystickChannel = 0;

  frc::Talon m_frontLeft{kFrontLeftChannel};
  frc::Talon m_rearLeft{kRearLeftChannel};
  frc::Talon m_frontRight{kFrontRightChannel};
  frc::Talon m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};

  frc::Joystick m_stick{kJoystickChannel};
  const double kSlowSpeedFactor = 1.7;
  double speed_factor = kSlowSpeedFactor;

  AHRS *ahrs;  // navX MXP (update for Magic House April 2019)

public:
  void RobotInit() override {
    // Invert the left side motors. You may need to change or remove this to
    // match your robot.
    m_frontLeft.SetInverted(true);
    m_rearLeft.SetInverted(true);
  }

   Robot() {
    try {
          ahrs = new AHRS(SPI::Port::kMXP);
    } catch (std::exception& ex ) {
          std::string err_string = "Error instantiating navX MXP:  ";
          err_string += ex.what();
          DriverStation::ReportError(err_string.c_str());
    }
  }

  void TeleopPeriodic() override {
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
    // pilot controls
    bool slow_gear_button_pressed = m_stick.GetRawButton(3);
    bool high_gear_button_presssed = m_stick.GetRawButton(2);
    if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
    else if (high_gear_button_presssed) {speed_factor = 1;}
    frc::SmartDashboard::PutNumber ("DB/gyro angle", -ahrs->GetAngle());
    bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
    if ( reset_yaw_button_pressed ) {
      ahrs->ZeroYaw();
    }
    m_robotDrive.DriveCartesian(-m_stick.GetX()/speed_factor, m_stick.GetRawAxis(3)/speed_factor, 
        m_stick.GetY()/speed_factor, -ahrs->GetAngle());
  }


};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
