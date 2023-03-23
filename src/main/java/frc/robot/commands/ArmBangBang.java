package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmBangBang extends CommandBase {
    ArmSubsystem arm;
    double target, tolerance, speed, failsafeLimit;
    boolean useSpeedDown, failsafe = true;

    /**
     * Creates a ArmBangBang Command. This command is used to move the arm to a
     * certain position using Bang Bang control.
     * 
     * @param arm           The arm subsystem
     * @param target        The target position
     * @param tolerance     The tolerance for the position
     * @param speed         The speed to move at
     * @param useSpeedDown  True to use the speed for moving down, false to disable
     *                      motors and use gravity
     * @param failsafeLimit Software limit on arm position to stop all arm movement.
     */
    public ArmBangBang(ArmSubsystem arm, double target, double tolerance, double speed, boolean useSpeedDown,
            double failsafeLimit) {
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
