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
    private final ArmSubsystem arm = new ArmSubsystem(Constants.ID.ARM);
    private final DriveSubsystem drive = new DriveSubsystem(
        Constants.ID.L1, Constants.ID.L2, Constants.ID.R1, Constants.ID.R2, Constants.Drive.LM_INVERSE, 
        Constants.Drive.RM_INVERSE, Constants.Drive.LM_SPEED_OFFSET, Constants.Drive.RM_SPEED_OFFSET
    );
    private final Joystick controller = new Joystick(Constants.ID.JOYSTICK);
    private final GyroSubsystem gyro = new GyroSubsystem(Constants.Gyro.USE_ROLL, Constants.Gyro.UPSIDE_DOWN);

    public RobotContainer() {
        configureButtonBindings();

        drive.setDefaultCommand(
            new TeleopDrive(drive, () -> controller.getRawAxis(Constants.TeleOp.LEFT_DRIVE_STICK), 
            () -> controller.getRawAxis(Constants.TeleOp.RIGHT_DRIVE_STICK),
            () -> controller.getRawButton(Constants.TeleOp.TURBO_BTN), Constants.TeleOp.MOVE_SCALE,
            Constants.TeleOp.TURN_SCALE)
        );
        arm.setDefaultCommand(
            (Constants.Arm.TYPE == Constants.ArmMovement.PID) ?
                new ArmPID(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.PID.P, Constants.Arm.PID.I, Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT) :
                new ArmBangBang(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.TOLERANCE, Constants.Arm.SPEED, Constants.Arm.USE_SPEED_REVERSE, Constants.Arm.LIMIT)
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    @SuppressWarnings("all") // false positives from use of config constants
    private void configureButtonBindings() {
        new JoystickButton(controller, Constants.Arm.RESET_BTN).onTrue(
            Commands.runOnce(arm::resetInitialPos)
        );
        new JoystickButton(controller, Constants.Arm.OVERRIDE_BTN).whileTrue(
            Commands.startEnd(() -> arm.setSpeed(Constants.Arm.OVERRIDE_SPEED), () -> arm.setSpeed(0), arm)
        );
        new JoystickButton(controller, Constants.Arm.OVERRIDE_REVERSE_BTN).whileTrue(
            Commands.startEnd(() -> arm.setSpeed(Constants.Arm.OVERRIDE_REVERSE_SPEED), () -> arm.setSpeed(0), arm)
        );
        new JoystickButton(controller, Constants.Arm.PS_LOW_BTN).onTrue(
            (Constants.Arm.TYPE == Constants.ArmMovement.PID) ?
                new ArmPID(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.PID.P, Constants.Arm.PID.I, Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT) :
                new ArmBangBang(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.TOLERANCE, Constants.Arm.SPEED, Constants.Arm.USE_SPEED_REVERSE, Constants.Arm.LIMIT)
        );
        new JoystickButton(controller, Constants.Arm.PS_HIGH_BTN).onTrue(
            (Constants.Arm.TYPE == Constants.ArmMovement.PID) ?
                new ArmPID(arm, Constants.Arm.PS_HIGH_POS, Constants.Arm.PID.P, Constants.Arm.PID.I, Constants.Arm.PID.D, Constants.Arm.PID.I_TOLERANCE, Constants.Arm.LIMIT) :
                new ArmBangBang(arm, Constants.Arm.PS_LOW_POS, Constants.Arm.TOLERANCE, Constants.Arm.SPEED, Constants.Arm.USE_SPEED_REVERSE, Constants.Arm.LIMIT)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @SuppressWarnings("all") // false positives from use of config constants
    public Command getAutonomousCommand() {
        switch (Constants.Autonomous.SEQUENCE) {
            case DOCK:
                return new SequentialCommandGroup(
                    Commands.waitUntil(gyro::isReady),
                    Commands.runOnce(gyro::zeroYaw),
                    new DriveUntilPitch(drive, gyro, Constants.Autonomous.MOVE_SPEED, 0, 3, true),
                    new DockPID(drive, gyro, Constants.Autonomous.DockPID.P, Constants.Autonomous.DockPID.I, 
                        Constants.Autonomous.DockPID.D, Constants.Autonomous.DockPID.I_TOLERANCE)
                );
            default: return null;
        }
    }
}
