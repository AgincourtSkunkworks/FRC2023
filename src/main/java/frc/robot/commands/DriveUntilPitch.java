package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveUntilPitch extends CommandBase {
    DriveSubsystem drive;
    GyroSubsystem gyro;
    double speed, target, tolerance, maxTemp;
    boolean not, failsafe = false;

    /**
     * Creates a DriveUntilPitch Command. This command is used to drive at a certain
     * speed until the robot reaches (or leaves) a certain pitch.
     * 
     * @param drive     The drive subsystem
     * @param gyro      The gyro subsystem
     * @param speed     The speed to drive at
     * @param target    The target pitch
     * @param tolerance The tolerance for the pitch
     * @param not       True to drive until pitch is out of range, false to drive
     *                  until pitch is in range
     * @param maxTemp   The maximum temperature before thermal failsafe kicks in
     */
    public DriveUntilPitch(DriveSubsystem drive, GyroSubsystem gyro, double speed, double target, double tolerance,
            boolean not, double maxTemp) {
        addRequirements(drive);
        this.drive = drive;
        this.speed = speed;
        this.gyro = gyro;
        this.target = target;
        this.tolerance = tolerance;
        this.not = not;
        this.maxTemp = maxTemp;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Command", "DriveUntilPitch");
        this.failsafe = false;
    }

    @Override
    public void execute() {
        if (drive.getHighestTemp() >= maxTemp) {
            failsafe = true;
            System.out.println("[DriveUntilPitch] Failsafe activated, high motor temperature!]");
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
        final boolean finished = Math.abs(gyro.getVAngle() - target) <= tolerance;
        return this.failsafe || (not ? !finished : finished);
    }
}
