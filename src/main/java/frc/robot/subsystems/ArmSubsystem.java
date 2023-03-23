// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    TalonFX armMotor;
    double initialPos;
    private boolean override = false;

    /**
     * Creates a new ArmSubsystem with brake mode enabled.
     * 
     * @param armMotorID ID of the arm motor controller
     */
    public ArmSubsystem(int armMotorID) {
        armMotor = new TalonFX(armMotorID);
        initialPos = armMotor.getSelectedSensorPosition();
        armMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Creates a new ArmSubsystem.
     * 
     * @param armMotorID ID of the arm motor controller
     * @param armInvert  Whether to invert the arm motor
     * @param brakeMode  Whether to use brake mode
     */
    public ArmSubsystem(int armMotorID, boolean armInvert, boolean brakeMode) {
        armMotor = new TalonFX(armMotorID);
        initialPos = armMotor.getSelectedSensorPosition();
        if (armInvert)
            armMotor.setInverted(true);
        if (brakeMode)
            armMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Set the speed of the arm motor.
     * 
     * @param speed Speed to set the motor to (-1 to 1)
     */
    public void setSpeed(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Get the position of the arm motor. Offset by the initial position.
     * 
     * @return The offset position of the arm motor in raw encoder units
     */
    public double getArmPos() {
        return armMotor.getSelectedSensorPosition() - initialPos;
    }

    /**
     * Get the position of the arm motor without offset
     * 
     * @return The position of the arm motor in raw encoder units
     */
    public double getArmPosRaw() {
        return armMotor.getSelectedSensorPosition();
    };

    /**
     * Resets the initial position (used for offset) of the arm motor
     */
    public void resetInitialPos() {
        this.initialPos = this.getArmPosRaw();
    }

    /**
     * Get whether the arm is currently being manually overridden
     * 
     * @return Whether the arm is currently being overridden
     */
    public boolean isOverridden() {
        return override;
    }

    /**
     * Set whether the arm is currently being manually overridden
     * 
     * @param override Whether the arm is currently being overridden
     */
    public void setOverride(boolean override) {
        this.override = override;
    }

    /**
     * Enable override mode on the arm
     */
    public void override() {
        this.setOverride(true);
    }

    /**
     * Disable override mode on the arm
     */
    public void reenable() {
        this.setOverride(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getArmPos());
        SmartDashboard.putNumber("Arm Position Raw", getArmPosRaw());
    }
}
