/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <wpi/raw_ostream.h>

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together.
 */
class Robot : public frc::TimedRobot {
 public:
 void RobotInit() {
    srx.Set(ControlMode::PercentOutput, 0); 
    wpi::outs() << "hello from RobotInit\n";
    }

  void TeleopPeriodic() override { 
    int dpad_angle = m_stick.GetPOV(0);
    if (dpad_angle != -1) {
      state = 1;
      wpi::outs() << "d pad pressed\n"; wpi::outs() << dpad_angle; wpi::outs() << "\n";
    } else if (m_stick.GetRawButton(1) || m_stick.GetRawButton(2) 
            || m_stick.GetRawButton(3) || m_stick.GetRawButton(4)) {
      state = 0;
    }
    switch (state) {
      case 0:
        speed = m_stick.GetY();
        break;
      case 1:
        switch (dpad_angle) {
          case 0: speed = .25; break;
          case 90: speed = .5; break;
          case 180: speed = .75; break;
          case 270: speed = 1; break;
        }
        break;
    }
    srx.Set(speed);
  }

 private:
  frc::Joystick m_stick{0};
  WPI_TalonSRX srx = {6};
  int state = 0;
  double speed = 0.0;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
