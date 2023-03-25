// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    // Create required systems
    private final ArmSubsystem arm = new ArmSubsystem(Constants.ID.ARM);
    private final DriveSubsystem drive = new DriveSubsystem(
            Constants.ID.L1, Constants.ID.L2, Constants.ID.R1, Constants.ID.R2, Constants.Drive.LM_INVERSE,
            Constants.Drive.RM_INVERSE, Constants.Drive.LM_SPEED_OFFSET, Constants.Drive.RM_SPEED_OFFSET,
            Constants.Drive.BRAKE_THRESHOLD, Constants.Drive.THERMAL_WARNING);
    private final Joystick controller = new Joystick(Constants.ID.JOYSTICK);
    private final GyroSubsystem gyro = new GyroSubsystem(Constants.Gyro.USE_ROLL, Constants.Gyro.UPSIDE_DOWN);

    public RobotContainer() {
        configureButtonBindings();

        drive.setDefaultCommand(
                new TeleopDrive(drive, () -> -controller.getRawAxis(Constants.TeleOp.LEFT_DRIVE_STICK),
                        () -> -controller.getRawAxis(Constants.TeleOp.RIGHT_DRIVE_STICK),
                        () -> controller.getRawButton(Constants.TeleOp.TURBO_BTN), Constants.TeleOp.MOVE_SCALE,
                        Constants.TeleOp.TURN_SCALE));
        arm.setDefaultCommand(
                (Constants.Arm.TYPE == Constants.ArmMovement.PID)
                        ? new ArmPID(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                                Constants.Arm.PID.I,
                                Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT)
                        : new ArmBangBang(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.TOLERANCE, Constants.Arm.SPEED,
                                Constants.Arm.REVERSE_SPEED, Constants.Arm.LIMIT));
    }

    @SuppressWarnings("all") // false positives from use of config constants
    private void configureButtonBindings() {
        new JoystickButton(controller, Constants.Arm.RESET_BTN).onTrue(
                Commands.runOnce(arm::resetInitialPos));
        new JoystickButton(controller, Constants.Arm.OVERRIDE_BTN).onTrue(Commands.runOnce(arm::override)).whileTrue(
                Commands.startEnd(() -> arm.setSpeed(Constants.Arm.OVERRIDE_SPEED), () -> arm.setSpeed(0), arm));
        new JoystickButton(controller, Constants.Arm.OVERRIDE_REVERSE_BTN).onTrue(Commands.runOnce(arm::override))
                .whileTrue(
                        Commands.startEnd(() -> arm.setSpeed(Constants.Arm.OVERRIDE_REVERSE_SPEED),
                                () -> arm.setSpeed(0),
                                arm));
        new JoystickButton(controller, Constants.Arm.PS_LOW_BTN).onTrue(Commands.runOnce(arm::reenable)).onTrue(
                (Constants.Arm.TYPE == Constants.ArmMovement.PID)
                        ? new ArmPID(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                                Constants.Arm.PID.I,
                                Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT)
                        : new ArmBangBang(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.TOLERANCE, Constants.Arm.SPEED,
                                Constants.Arm.REVERSE_SPEED, Constants.Arm.LIMIT));
        new JoystickButton(controller, Constants.Arm.PS_HIGH_BTN).onTrue(Commands.runOnce(arm::reenable)).onTrue(
                (Constants.Arm.TYPE == Constants.ArmMovement.PID)
                        ? new ArmPID(arm, Constants.Arm.PS_HIGH_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                                Constants.Arm.PID.I,
                                Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT)
                        : new ArmBangBang(arm, Constants.Arm.PS_HIGH_POS, Constants.Arm.TOLERANCE, Constants.Arm.SPEED,
                                Constants.Arm.REVERSE_SPEED, Constants.Arm.LIMIT));
    }

    @SuppressWarnings("all") // false positives from use of config constants
    public Command getAutonomousCommand() {
        switch (Constants.Autonomous.SEQUENCE) {
            case LEAVE:
                return new SequentialCommandGroup(
                        Commands.waitUntil(gyro::isReady),
                        Commands.runOnce(gyro::zeroYaw),
                        new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveUntilPitch(drive, gyro,
                                Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.DOWN_SCALE, 0, 3, false,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED, Constants.Autonomous.COMM_LEAVE_TIME,
                                Constants.Autonomous.MAX_TEMP));
            case DOCK:
                return new SequentialCommandGroup(
                        Commands.waitUntil(gyro::isReady),
                        Commands.runOnce(gyro::zeroYaw),
                        new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true,
                                Constants.Autonomous.MAX_TEMP),
                        new DockPID(drive, gyro, Constants.Autonomous.DockPID.P, Constants.Autonomous.DockPID.I,
                                Constants.Autonomous.DockPID.D, Constants.Autonomous.DockPID.I_TOLERANCE,
                                Constants.Autonomous.MAX_TEMP));
            case LEAVE_DOCK: // TODO: Test LEAVE_DOCK
                return new SequentialCommandGroup(
                        Commands.waitUntil(gyro::isReady),
                        Commands.runOnce(gyro::zeroYaw),
                        new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveUntilPitch(drive, gyro,
                                Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.DOWN_SCALE, 0, 3, false,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED, Constants.Autonomous.COMM_LEAVE_TIME,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveUntilPitch(drive, gyro, -Constants.Autonomous.MOVE_SPEED, 0, 7, true,
                                Constants.Autonomous.MAX_TEMP),
                        new DockPID(drive, gyro,
                                Constants.Autonomous.DockPIDReverse.P
                                        * Constants.Autonomous.DockPIDReverse.P_REVERSE_SCALE,
                                Constants.Autonomous.DockPIDReverse.I,
                                Constants.Autonomous.DockPIDReverse.D, Constants.Autonomous.DockPIDReverse.I_TOLERANCE,
                                Constants.Autonomous.MAX_TEMP));
            case ARM_LEAVE_DOCK: // TODO: Test ARM_LEAVE_DOCK
                return new SequentialCommandGroup(
                        Commands.waitUntil(gyro::isReady),
                        Commands.runOnce(gyro::zeroYaw),
                        Commands.runOnce(arm::resetInitialPos),
                        Commands.runOnce(arm::reenable),
                        new ArmPID(arm, Constants.Arm.PS_HIGH_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                                Constants.Arm.PID.I, Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE,
                                Constants.Arm.LIMIT),
                        new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveUntilPitch(drive, gyro,
                                Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.DOWN_SCALE, 0, 3, false,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED, Constants.Autonomous.COMM_LEAVE_TIME,
                                Constants.Autonomous.MAX_TEMP),
                        new DriveUntilPitch(drive, gyro, -Constants.Autonomous.MOVE_SPEED, 0, 7, true,
                                Constants.Autonomous.MAX_TEMP),
                        new DockPID(drive, gyro,
                                Constants.Autonomous.DockPIDReverse.P
                                        * Constants.Autonomous.DockPIDReverse.P_REVERSE_SCALE,
                                Constants.Autonomous.DockPIDReverse.I,
                                Constants.Autonomous.DockPIDReverse.D, Constants.Autonomous.DockPIDReverse.I_TOLERANCE,
                                Constants.Autonomous.MAX_TEMP));
            case NONE:
                return null;
            default:
                System.out.println("[ERROR] Invalid autonomous sequence");
                return null;
        }
    }
}
