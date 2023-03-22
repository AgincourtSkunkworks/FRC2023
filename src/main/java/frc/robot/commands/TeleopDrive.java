package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends CommandBase {
    DriveSubsystem drive;
    double driveScale, turnScale;
    Supplier<Double> lSpeedFunc, rSpeedFunc;
    Supplier<Boolean> offsetOverride;

    public TeleopDrive(DriveSubsystem drive, Supplier<Double> lSpeedFunc, Supplier<Double> rSpeedFunc, Supplier<Boolean> offsetOverride, double driveScale, double turnScale) {
        addRequirements(drive);
        this.drive = drive;
        this.lSpeedFunc = lSpeedFunc;
        this.rSpeedFunc = rSpeedFunc;
        this.offsetOverride = offsetOverride;
        this.driveScale = driveScale;
        this.turnScale = turnScale;
    }

    @Override
    public void execute() {
        final double stickLeft = lSpeedFunc.get();
        final double stickRight = rSpeedFunc.get();
        if (stickLeft > 0.05 && stickRight < 0.05 || stickLeft < 0.05 && stickRight > 0.05) { // no movement is ~+-0.007, not absolute zero
            drive.setLeftMotors(stickLeft * ((offsetOverride.get()) ? 1 : driveScale));
            drive.setRightMotors(stickRight * ((offsetOverride.get()) ? 1 : driveScale));
        } else {
            drive.setLeftMotors(stickLeft * ((offsetOverride.get()) ? 1 : turnScale));
            drive.setRightMotors(stickRight * ((offsetOverride.get()) ? 1 : turnScale));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
