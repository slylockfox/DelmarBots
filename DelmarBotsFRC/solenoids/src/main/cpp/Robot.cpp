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
#include <frc/smartdashboard/SmartDashboard.h>
//#include <iostream>
#include <wpi/raw_ostream.h>

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
  void RobotInit() override {
    wpi::outs() << "hello from RobotInit\n";
  }

  void TeleopInit() override { }

  void TeleopPeriodic() override {
    /* The output of GetRawButton is true/false depending on whether the button
     * is pressed; Set takes a boolean for whether to use the default
     * (false) channel or the other (true).
     */
    // m_solenoid4.Set(m_stick.GetRawButton(kSolenoidButton0));
    // m_solenoid5.Set(m_stick.GetRawButton(kSolenoidButton1));

    /* In order to set the double solenoid, we will say that if neither button
     * is pressed, it is off, if just one button is pressed, set the solenoid to
     * correspond to that button, and if both are pressed, set the solenoid to
     * Forwards.
     */
    wpi::outs() << "teleop loop";
    frc::SmartDashboard::PutNumber ("DB/logtest", 1);
    if (m_stick.GetRawButton(kDoubleSolenoidForward)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
      frc::SmartDashboard::PutString ("DB/solonoid", "forward");
    } else if (m_stick.GetRawButton(kDoubleSolenoidReverse)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
      frc::SmartDashboard::PutString ("DB/solonoid", "reverse");
    } else {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
      frc::SmartDashboard::PutString ("DB/solonoid", "off");
    }
  }

 private:
  frc::Joystick m_stick{0};

  // Solenoid corresponds to a single solenoid.
  frc::Solenoid m_solenoid0{0};
  frc::Solenoid m_solenoid1{1};
  frc::Solenoid m_solenoid2{2};
  frc::Solenoid m_solenoid3{3};
  // frc::Solenoid m_solenoid4{4};
  // frc::Solenoid m_solenoid5{5};

  // DoubleSolenoid corresponds to a double solenoid.
   frc::DoubleSolenoid m_doubleSolenoid{4,5};

  // joystick buttons
  static constexpr int kSolenoidButton0 = 1;
  static constexpr int kSolenoidButton1 = 2;
  static constexpr int kSolenoidButton2 = 3;
  static constexpr int kSolenoidButton3 = 4;
  static constexpr int kSolenoidButton4 = 5;
  static constexpr int kSolenoidButton5 = 6;
   static constexpr int kDoubleSolenoidForward = 1;
   static constexpr int kDoubleSolenoidReverse = 2;
};

#ifndef RUNNING_FRC_TESTS
int main() { 
  return frc::StartRobot<Robot>(); }
#endif
