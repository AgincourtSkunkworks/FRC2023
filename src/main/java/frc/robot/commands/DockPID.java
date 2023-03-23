package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DockPID extends PIDCommand {
    private boolean allowFinish;

    public DockPID(DriveSubsystem drive, GyroSubsystem gyro, double p, double i, double d) {
        super(
            new PIDController(p, i, d),
            gyro::getVAngle,
            0,
            output -> drive.setMotors(output),
            drive
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
