// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            Constants.ID.LM1, Constants.ID.LM2, Constants.ID.RM1, Constants.ID.RM2, Constants.Drive.LM_INVERSE,
            Constants.Drive.RM_INVERSE, Constants.Drive.LM_SPEED_OFFSET, Constants.Drive.RM_SPEED_OFFSET,
            Constants.Drive.BRAKE_THRESHOLD, Constants.Drive.THERMAL_WARNING, Constants.Drive.CurrentLimit.SUPPLY,
            Constants.Drive.CurrentLimit.SUPPLY_LIMIT, Constants.Drive.CurrentLimit.SUPPLY_TRIGGER,
            Constants.Drive.CurrentLimit.SUPPLY_TRIGGER_TIME, Constants.Drive.CurrentLimit.STATOR,
            Constants.Drive.CurrentLimit.STATOR_LIMIT, Constants.Drive.CurrentLimit.STATOR_TRIGGER,
            Constants.Drive.CurrentLimit.STATOR_TRIGGER_TIME, Constants.Autonomous.MAX_TEMP);
    private final Joystick controller = new Joystick(Constants.ID.JOYSTICK);
    private final GyroSubsystem gyro = new GyroSubsystem(Constants.Gyro.USE_ROLL, Constants.Gyro.UPSIDE_DOWN);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureButtonBindings();

        drive.setDefaultCommand(
                new TeleopDrive(drive, () -> -controller.getRawAxis(Constants.TeleOp.LEFT_DRIVE_STICK),
                        () -> -controller.getRawAxis(Constants.TeleOp.RIGHT_DRIVE_STICK),
                        Constants.TeleOp.SLEW_RATE_LIMIT));
        arm.setDefaultCommand(new ArmPID(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                Constants.Arm.PID.I,
                Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT));

        // NOTE: Should probably create some functions or just an array of different segments to prevent code duplication, but
        //      on the off chance a particular segment requires value adjusting, it's easier to have it like this for on the fly
        //      adjustments.
        autoChooser.setDefaultOption("Dock", new SequentialCommandGroup(
                Commands.waitUntil(gyro::isReady),
                Commands.runOnce(gyro::zeroYaw),
                new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true),
                new DockPID(drive, gyro, Constants.Autonomous.DockPID.P, Constants.Autonomous.DockPID.I,
                        Constants.Autonomous.DockPID.D, Constants.Autonomous.DockPID.I_TOLERANCE)));
        autoChooser.addOption("Leave (Charging Station)", new SequentialCommandGroup(
                Commands.waitUntil(gyro::isReady),
                Commands.runOnce(gyro::zeroYaw),
                new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true),
                new DriveUntilPitch(drive, gyro,
                        Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.DOWN_SCALE, 0, 3, false),
                new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED, Constants.Autonomous.COMM_LEAVE_TIME)));
        autoChooser.addOption("Leave (Straight)", new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED,
                Constants.Autonomous.COMM_LEAVE_STRAIGHT_TIME));
        autoChooser.addOption("Arm & Leave (Straight)", new SequentialCommandGroup(
                Commands.runOnce(arm::resetInitialPos),
                Commands.runOnce(arm::reenable),
                new ArmPID(arm, Constants.Arm.PS_HIGH_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                        Constants.Arm.PID.I, Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE,
                        Constants.Arm.LIMIT, Constants.Autonomous.ARM_TIME),
                new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED,
                        Constants.Autonomous.COMM_LEAVE_STRAIGHT_TIME)));
        autoChooser.addOption("Leave & Dock", new SequentialCommandGroup(
                Commands.waitUntil(gyro::isReady),
                Commands.runOnce(gyro::zeroYaw),
                new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true),
                new DriveUntilPitch(drive, gyro,
                        Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.DOWN_SCALE, 0, 3, false),
                new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED, Constants.Autonomous.COMM_LEAVE_TIME),
                new DriveUntilPitch(drive, gyro, -Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.REV_SCALE, 0,
                        7, true),
                new DockPID(drive, gyro,
                        Constants.Autonomous.DockPIDReverse.P,
                        Constants.Autonomous.DockPIDReverse.I,
                        Constants.Autonomous.DockPIDReverse.D, Constants.Autonomous.DockPIDReverse.I_TOLERANCE)));
        autoChooser.addOption("Arm & Leave & Dock", new SequentialCommandGroup(
                Commands.waitUntil(gyro::isReady),
                Commands.runOnce(gyro::zeroYaw),
                Commands.runOnce(arm::resetInitialPos),
                Commands.runOnce(arm::reenable),
                new ArmPID(arm, Constants.Arm.PS_HIGH_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                        Constants.Arm.PID.I, Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE,
                        Constants.Arm.LIMIT, Constants.Autonomous.ARM_TIME),
                new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true),
                new DriveUntilPitch(drive, gyro,
                        Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.DOWN_SCALE, 0, 3, false),
                new DriveForTime(drive, Constants.Autonomous.MOVE_SPEED, Constants.Autonomous.COMM_LEAVE_TIME),
                new DriveUntilPitch(drive, gyro, -Constants.Autonomous.MOVE_SPEED * Constants.Autonomous.REV_SCALE, 0,
                        7, true),
                new DockPID(drive, gyro,
                        Constants.Autonomous.DockPIDReverse.P,
                        Constants.Autonomous.DockPIDReverse.I,
                        Constants.Autonomous.DockPIDReverse.D, Constants.Autonomous.DockPIDReverse.I_TOLERANCE)));
        autoChooser.addOption("None", null);

        SmartDashboard.putData(autoChooser);
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
                new ArmPID(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                        Constants.Arm.PID.I,
                        Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT));
        new JoystickButton(controller, Constants.Arm.PS_HIGH_BTN).onTrue(Commands.runOnce(arm::reenable)).onTrue(
                new ArmPID(arm, Constants.Arm.PS_HIGH_POS, Constants.Arm.PID.P, Constants.Arm.PID.P_D,
                        Constants.Arm.PID.I,
                        Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
