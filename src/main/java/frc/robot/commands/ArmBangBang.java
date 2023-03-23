package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmBangBang extends CommandBase {
    ArmSubsystem arm;
    double target, tolerance, speed, failsafeLimit;
    boolean useSpeedDown, failsafe = true;

    /** Create an ArmBangBang command, which is used to automatically turn the arm with Bang Bang control.
     * @param arm Arm Subsystem
     * @param target Target position in raw encoder ticks (assuming 0 is lowest position)
     * @param tolerance 
     * @param speed Speed to turn the motor
     * @param useSpeedDown Whether to use negative speed to go down, otherwise turn off the motor and slowly descend
     * @param failsafeLimit Hard limit for encoder position to stop all movement at
     */
    public ArmBangBang(ArmSubsystem arm, double target, double tolerance, double speed, boolean useSpeedDown, double failsafeLimit) {
        addRequirements(arm);
        this.target = target;
        this.tolerance = tolerance;
        this.speed = speed;
        this.useSpeedDown = useSpeedDown;
        this.failsafeLimit = failsafeLimit;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Command", "ArmBangBang");
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
        final double armCurPos = arm.getArmPos();
        if (armCurPos < target - tolerance)
            arm.setSpeed(this.speed);
        else if (armCurPos > target + tolerance)
            arm.setSpeed((this.useSpeedDown) ? -this.speed : 0);
        else
            arm.setSpeed(0);
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
