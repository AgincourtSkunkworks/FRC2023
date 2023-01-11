// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  TalonSRX rightMotor1 = new TalonSRX(8);
  TalonSRX rightMotor2 = new TalonSRX(7);
  TalonSRX rightMotor3 = new TalonSRX(6);
  TalonSRX leftMotor1 = new TalonSRX(0);
  TalonSRX leftMotor2 = new TalonSRX(1);
  TalonSRX leftMotor3 = new TalonSRX(2);

  TalonSRX[] motors = { rightMotor1, rightMotor2, rightMotor3, leftMotor1, leftMotor3, leftMotor2};
  // TalonSRX[] leftmotors = {leftMotor1, leftMotor2, leftMotor3}

  Joystick j = new Joystick(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    for (TalonSRX motor : motors) {
      motor.configPeakCurrentLimit(45); // once at 60 (40 previously) amps, limit current
      motor.configPeakCurrentDuration(80); // for 80ms
      motor.configContinuousCurrentLimit(25); // set to 35 (20 previously) amps which will make robot slower
      motor.enableCurrentLimit(true); // enable
    }
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double stickLeft = j.getRawAxis(1);
    double stickRight = -j.getRawAxis(3);
    leftMotor1.set(ControlMode.PercentOutput, stickLeft);
    leftMotor2.set(ControlMode.PercentOutput, stickLeft);
    leftMotor3.set(ControlMode.PercentOutput, stickLeft);
    rightMotor1.set(ControlMode.PercentOutput, stickRight);
    rightMotor2.set(ControlMode.PercentOutput, stickRight);
    rightMotor3.set(ControlMode.PercentOutput, stickRight);
  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
