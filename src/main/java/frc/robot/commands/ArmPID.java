package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

// WPILib has a PIDCommand class, but I prefer to know exactly what the code is doing (for easy debugging)
// TODO: Create custom parent class for PID commands
public class ArmPID extends CommandBase {
    ArmSubsystem arm;
    double target, p, i, d, lastRunTime, iTolerance, errorSum, lastError, failsafeLimit;
    boolean hasITolerance, failsafe = false;

    /** Creates a ArmPID Command. This command is used to move the arm to a specific position, using PID control.
     * @param arm The arm subsystem
     * @param target The target position
     * @param p The P value of PID
     * @param i The I value of PID
     * @param d The D value of PID
     * @param iTolerance Under this tolerance, the errorSum will be increased. Set to 0 to disable.
     * @param failsafeLimit Software limit on arm position to stop all arm movement.
     */
    public ArmPID(ArmSubsystem arm, double target, double p, double i, double d, double iTolerance,
            double failsafeLimit) {
        addRequirements(arm);
        this.arm = arm;
        this.target = target;
        this.p = p;
        this.i = i;
        this.d = d;
        this.iTolerance = iTolerance;
        this.failsafeLimit = failsafeLimit;
        this.hasITolerance = iTolerance != 0;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Command", "ArmPID");
        this.errorSum = 0;
        this.lastRunTime = 0;
        this.lastError = 0;
        this.failsafe = false;
    }

    @Override
    public void execute() {
        if (this.failsafe || (failsafeLimit < 0 && arm.getArmPos() <= failsafeLimit)
                || (failsafeLimit > 0 && arm.getArmPos() >= failsafeLimit)) {
            System.out.println("[ARM] Failsafe Limit Hit! Arm Movement Stopped");
            this.failsafe = true;
            arm.setSpeed(0);
            return;
        }
        final double curTime = Timer.getFPGATimestamp();
        final double dt = curTime - this.lastRunTime;
        final double error = target - arm.getArmPos();
        final double errorRate = (error - this.lastError) / dt;
        if (!this.hasITolerance || Math.abs(error) <= this.iTolerance)
            this.errorSum += error * dt;
        arm.setSpeed((error * this.p) + (errorSum * this.i) + (errorRate * this.d));
        this.lastRunTime = curTime;
        this.lastError = error;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setSpeed(0);
        SmartDashboard.putString("Command", "None");
    }

    @Override
    public boolean isFinished() {
        return this.failsafe;
    }
}
