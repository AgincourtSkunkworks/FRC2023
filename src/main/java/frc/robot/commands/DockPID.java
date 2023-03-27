package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

// WPILib has a PIDCommand class, but I prefer to know exactly what the code is doing (for easy debugging)
// TODO: Create custom parent class for PID commands
public class DockPID extends CommandBase {
    DriveSubsystem drive;
    GyroSubsystem gyro;
    double p, i, d, lastRunTime, iTolerance, errorSum, lastError, maxTemp;
    boolean hasITolerance, failsafe = false;

    /**
     * Creates a DockPID Command. This command is used to drive until the robot is
     * docked, using PID control.
     * 
     * @param drive      The drive subsystem
     * @param gyro       The gyro subsystem
     * @param p          The P value of PID
     * @param i          The I value of PID
     * @param d          The D value of PID
     * @param iTolerance Under this tolerance, the errorSum will be increased. Set
     *                   to 0 to disable.
     */
    public DockPID(DriveSubsystem drive, GyroSubsystem gyro, double p, double i, double d, double iTolerance) {
        addRequirements(drive);
        this.drive = drive;
        this.gyro = gyro;
        this.p = p;
        this.i = i;
        this.d = d;
        this.iTolerance = iTolerance;
        this.hasITolerance = iTolerance != 0;
        this.maxTemp = drive.getMaxTemp();
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Command", "DockPID");
        this.errorSum = 0;
        this.lastRunTime = 0;
        this.lastError = 0;
        this.failsafe = false;
    }

    @Override
    public void execute() {
        if (drive.getHighestTemp() >= maxTemp) {
            failsafe = true;
            System.out.println("[DockPID] Failsafe activated, high motor temperature!]");
            return;
        }
        final double curTime = Timer.getFPGATimestamp();
        final double dt = curTime - this.lastRunTime;
        final double error = gyro.getVAngle();
        final double errorRate = (error - this.lastError) / dt;
        if (!this.hasITolerance || Math.abs(error) <= this.iTolerance)
            this.errorSum += error * dt;
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
        return this.failsafe;
    }
}
