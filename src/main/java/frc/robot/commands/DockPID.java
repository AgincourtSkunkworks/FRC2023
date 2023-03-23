package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DockPID extends PIDCommand {
    private boolean allowFinish;

    public DockPID(DriveSubsystem drive, GyroSubsystem gyro, double target, double k, double i, double d, boolean allowFinish) {
        super(
            new PIDController(k, i, d),
            gyro::getVAngle,
            target,
            output -> drive.setMotors(output),
            drive
        );
        this.allowFinish = allowFinish;
    }

    @Override
    public boolean isFinished() {
        return allowFinish && getController().atSetpoint();
    }
}
