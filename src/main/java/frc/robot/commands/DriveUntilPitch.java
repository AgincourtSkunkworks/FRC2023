package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveUntilPitch extends CommandBase {
    DriveSubsystem drive;
    GyroSubsystem gyro;
    double speed, target, tolerance;
    boolean not;

    public DriveUntilPitch(DriveSubsystem drive, GyroSubsystem gyro, double speed, double target, double tolerance, boolean not) {
        addRequirements(drive);
        this.drive = drive;
        this.speed = speed;
        this.gyro = gyro;
        this.target = target;
        this.tolerance = tolerance;
        this.not = not;
    }

    @Override
    public void initialize() {SmartDashboard.putString("Command", "DriveUntilPitch");}

    @Override
    public void execute() {drive.setMotors(speed);}

    @Override
    public void end(boolean interrupted) {
        drive.setMotors(0);
        SmartDashboard.putString("Command", "None");
    }

    @Override
    public boolean isFinished() {
        final boolean finished = Math.abs(gyro.getVAngle() - target) <= tolerance;
        return not ? !finished : finished;
    }
}
