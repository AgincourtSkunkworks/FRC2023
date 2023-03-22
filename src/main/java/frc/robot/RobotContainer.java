// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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
            new TeleopDrive(drive, () -> controller.getRawAxis(1), () -> controller.getRawAxis(3),
            () -> controller.getRawButton(8), Constants.TeleOp.MOVE_SCALE, Constants.TeleOp.TURN_SCALE)
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(

        );
    }
}
