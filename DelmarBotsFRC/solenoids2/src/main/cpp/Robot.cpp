/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/TimedRobot.h>

/**
 * This is a sample program showing the use of the solenoid classes during
 * operator control.
 *
 * Three buttons from a joystick will be used to control two solenoids: One
 * button to control the position of a single solenoid and the other two buttons
 * to control a double solenoid.
 *
 * Single solenoids can either be on or off, such that the air diverted through
 * them goes through either one channel or the other.
 *
 * Double solenoids have three states: Off, Forward, and Reverse. Forward and
 * Reverse divert the air through the two channels and correspond to the on and
 * off of a single solenoid, but a double solenoid can also be "off", where both
 * channels are diverted to exhaust such that there is no pressure in either
 * channel.
 *
 * Additionally, double solenoids take up two channels on your PCM whereas
 * single solenoids only take a single channel.
 */
class Robot : public frc::TimedRobot {
 public:
  void TeleopPeriodic() override {
    /* The output of GetRawButton is true/false depending on whether the button
     * is pressed; Set takes a boolean for for whether to use the default
     * (false) channel or the other (true).
     */
    m_solenoid.Set(m_stick.GetRawButton(kSolenoidButton));

    /* In order to set the double solenoid, we will say that if neither button
     * is pressed, it is off, if just one button is pressed, set the solenoid to
     * correspond to that button, and if both are pressed, set the solenoid to
     * Forwards.
     */
    if (m_stick.GetRawButton(kDoubleSolenoidForward)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else if (m_stick.GetRawButton(kDoubleSolenoidReverse)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
    } else {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
    }
  }

 private:
  frc::Joystick m_stick{0};

  // Solenoids
  frc::Solenoid m_solenoid{0};
  frc::DoubleSolenoid m_doubleSolenoid{4,5};

  // buttons
  static constexpr int kSolenoidButton = 1;
  static constexpr int kDoubleSolenoidForward = 2;
  static constexpr int kDoubleSolenoidReverse = 3;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
