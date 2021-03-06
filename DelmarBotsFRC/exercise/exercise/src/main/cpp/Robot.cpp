/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/Talon.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the left side motors. You may need to change or remove this to
    // match your robot.
    m_frontLeft.SetInverted(true);
    m_rearLeft.SetInverted(true);
  }

  void TeleopPeriodic() override {
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
    m_robotDrive.DriveCartesian(m_stick.GetX(), m_stick.GetY(), m_stick.GetZ());
  }

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
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
