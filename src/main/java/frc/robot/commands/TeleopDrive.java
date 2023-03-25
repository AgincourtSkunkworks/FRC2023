package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends CommandBase {
    DriveSubsystem drive;
    double driveScale, turnScale;
    Supplier<Double> lSpeedFunc, rSpeedFunc;
    Supplier<Boolean> offsetOverrideFunc;

    /**
     * Creates a TeleOpDrive Command. This command is used to control the drive in
     * teleop.
     * 
     * @param drive              The drive subsystem
     * @param lSpeedFunc         Function to get the speed to set the left motors to
     * @param rSpeedFunc         Function to get the speed to set the right motors
     *                           to
     * @param offsetOverrideFunc Function to get whether or not to override the
     *                           speed scaling
     * @param driveScale         The speed to scale the motors by when driving
     * @param turnScale          The speed to scale the motors by when turning
     */
    public TeleopDrive(DriveSubsystem drive, Supplier<Double> lSpeedFunc, Supplier<Double> rSpeedFunc,
            Supplier<Boolean> offsetOverrideFunc, double driveScale, double turnScale) {
        addRequirements(drive);
        this.drive = drive;
        this.lSpeedFunc = lSpeedFunc;
        this.rSpeedFunc = rSpeedFunc;
        this.offsetOverrideFunc = offsetOverrideFunc;
        this.driveScale = driveScale;
        this.turnScale = turnScale;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Command", "TeleopDrive");
    }

    @Override
    public void execute() {
        final double stickLeft = lSpeedFunc.get();
        final double stickRight = rSpeedFunc.get();
        final boolean offsetOverride = offsetOverrideFunc.get();
        if (offsetOverride) {
            drive.setLeftMotors(stickLeft);
            drive.setRightMotors(stickRight);
        } else if (stickLeft > 0.05 && stickRight < 0.05 || stickLeft < 0.05 && stickRight > 0.05) { // no movement is
                                                                                                     // ~+-0.007, not
                                                                                                     // absolute zero
            drive.setLeftMotors(stickLeft * driveScale);
            drive.setRightMotors(stickRight * driveScale);
        } else {
            drive.setLeftMotors(stickLeft * turnScale);
            drive.setRightMotors(stickRight * turnScale);
        }
        SmartDashboard.putNumber("Stick Left", stickLeft);
        SmartDashboard.putNumber("Stick Right", stickRight);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setMotors(0);
        SmartDashboard.putString("Command", "None");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
