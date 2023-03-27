package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForTime extends CommandBase {
    DriveSubsystem drive;
    double speed, time, startTime, maxTemp;
    boolean failsafe = false;

    /**
     * Creates a DriveForTime Command. This command is used to drive at a certain
     * speed for a certain time.
     * 
     * @param drive   The drive subsystem
     * @param speed   The speed to drive at
     * @param time    The time to drive for (in seconds)
     */
    public DriveForTime(DriveSubsystem drive, double speed, double time) {
        addRequirements(drive);
        this.drive = drive;
        this.speed = speed;
        this.time = time;
        this.maxTemp = drive.getMaxTemp();
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Command", "DriveForTime");
        this.startTime = Timer.getFPGATimestamp();
        this.failsafe = false;
        drive.setMotors(speed);
    }

    @Override
    public void execute() {
        if (drive.getHighestTemp() >= maxTemp) {
            failsafe = true;
            System.out.println("[DriveForTime] Failsafe activated, high motor temperature!]");
            return;
        }
        drive.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setMotors(0);
        SmartDashboard.putString("Command", "None");
    }

    @Override
    public boolean isFinished() {
        return this.failsafe || Timer.getFPGATimestamp() - startTime >= time;
    }
}
