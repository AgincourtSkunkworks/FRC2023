package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DockPIDCustom extends CommandBase {
    DriveSubsystem drive;
    GyroSubsystem gyro;
    double p, i, d, lastRunTime, iTolerance, errorSum, lastError;
    boolean allowFinish, hasITolerance;

    public DockPIDCustom(DriveSubsystem drive, GyroSubsystem gyro, double p, double i, double d) {
        addRequirements(drive);
        this.drive = drive;
        this.gyro = gyro;
        this.p = p;
        this.i = i;
        this.d = d;
        this.hasITolerance = false;
    }

    public DockPIDCustom(DriveSubsystem drive, GyroSubsystem gyro, double p, double i, double d, double iTolerance) {
        addRequirements(drive);
        this.drive = drive;
        this.gyro = gyro;
        this.p = p;
        this.i = i;
        this.d = d;
        this.iTolerance = iTolerance;
        this.hasITolerance = true;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Command", "DockPIDCustom");
        this.errorSum = 0;
    }

    @Override
    public void execute() {
        final double curTime = Timer.getFPGATimestamp();
        final double dt = curTime - this.lastRunTime;
        final double error = gyro.getVAngle();
        final double errorRate = (error - this.lastError) / dt;
        if (!this.hasITolerance || Math.abs(error) <= this.iTolerance) this.errorSum += error * dt;
        drive.setMotors((error * this.p) + (errorSum * this.i) + (errorRate * this.d));
        this.lastRunTime = curTime;
        this.lastError = error;
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
