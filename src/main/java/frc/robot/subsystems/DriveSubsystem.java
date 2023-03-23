package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    TalonFX leftMotor1, leftMotor2, rightMotor1, rightMotor2;
    TalonFX[] leftMotors, rightMotors, motors;
    double lCorrect, rCorrect;

    /**
     * Creates a new DriveSubsystem.
     * 
     * @param l1ID     Left Motor Controller ID
     * @param l2ID     Left Motor Controller ID
     * @param r1ID     Right Motor Controller ID
     * @param r2ID     Right Motor Controller ID
     * @param lInvert  Whether or not to invert the left motors
     * @param rInvert  Whether or not to invert the right motors
     * @param lCorrect Percent of speed to add to left motors (0-1)
     * @param rCorrect Percent of speed to add to right motors (0-1)
     */
    public DriveSubsystem(int l1ID, int l2ID, int r1ID, int r2ID, boolean lInvert, boolean rInvert, double lCorrect,
            double rCorrect) {
        leftMotor1 = new TalonFX(l1ID);
        leftMotor2 = new TalonFX(l2ID);
        rightMotor1 = new TalonFX(r1ID);
        rightMotor2 = new TalonFX(r2ID);

        leftMotors = new TalonFX[] { leftMotor1, leftMotor2 };
        rightMotors = new TalonFX[] { rightMotor1, rightMotor2 };
        motors = new TalonFX[] { leftMotor1, leftMotor2, rightMotor1, rightMotor2 };

        this.lCorrect = 1 + lCorrect;
        this.rCorrect = 1 + rCorrect;

        for (TalonFX motor : motors)
            motor.setNeutralMode(NeutralMode.Brake);

        if (lInvert)
            for (TalonFX motor : leftMotors)
                motor.setInverted(true);
        if (rInvert)
            for (TalonFX motor : rightMotors)
                motor.setInverted(true);
    }

    /**
     * Set the speed of the left motors.
     * 
     * @param speed Speed to set the motors to (-1 to 1)
     */
    public void setLeftMotors(double speed) {
        for (TalonFX motor : leftMotors)
            motor.set(ControlMode.PercentOutput, speed * lCorrect);
    }

    /**
     * Set the speed of the right motors.
     * 
     * @param speed Speed to set the motors to (-1 to 1)
     */
    public void setRightMotors(double speed) {
        for (TalonFX motor : rightMotors)
            motor.set(ControlMode.PercentOutput, speed * rCorrect);
    }

    /**
     * Set the speed of both motors.
     * 
     * @param speed Speed to set the motors to (-1 to 1)
     */
    public void setMotors(double speed) {
        setLeftMotors(speed);
        setRightMotors(speed);
    }

    /**
     * Get the position of the left motor. This uses left 1 as a reference,
     * both left motors should be very similar without hardware issues.
     * ! Add a number after Left to get the position of a specific motor.
     * 
     * @return The position of the left motors in raw encoder units.
     */
    public double getLeftPos() {
        return leftMotor1.getSelectedSensorPosition();
    }

    /**
     * Get the position of the right motor. This uses right 1 as a reference,
     * both right motors should be very similar without hardware issues.
     * ! Add a number after Right to get the position of a specific motor.
     * 
     * @return The position of the right motors in raw encoder units.
     */
    public double getRightPos() {
        return rightMotor1.getSelectedSensorPosition();
    }

    public double getLeft1Pos() {
        return leftMotor1.getSelectedSensorPosition();
    }

    public double getLeft2Pos() {
        return leftMotor2.getSelectedSensorPosition();
    }

    public double getRight1Pos() {
        return rightMotor1.getSelectedSensorPosition();
    }

    public double getRight2Pos() {
        return rightMotor2.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Motor 1 Position", getLeft1Pos());
        SmartDashboard.putNumber("Left Motor 2 Position", getLeft2Pos());
        SmartDashboard.putNumber("Right Motor 1 Position", getRight1Pos());
        SmartDashboard.putNumber("Right Motor 2 Position", getRight2Pos());
    }
}