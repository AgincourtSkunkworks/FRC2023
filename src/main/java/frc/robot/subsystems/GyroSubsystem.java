// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
    public AHRS gyroscopeAHRS;
    private boolean useRoll, upsideDown;

    /**
     * Creates a new ArmSubsystem with brake mode enabled.
     * 
     * @param armMotorID ID of the arm motor controller
     */
    public GyroSubsystem(boolean useRoll, boolean upsideDown) {
        gyroscopeAHRS = new AHRS(SPI.Port.kMXP);
        this.useRoll = useRoll;
        this.upsideDown = upsideDown;
    }

    /**
     * Check if the gyro is ready to be used
     * 
     * @return True if the gyro is ready, false otherwise.
     */
    public boolean isReady() {
        return gyroscopeAHRS.isConnected();
    }

    /**
     * Get the vertical angle of the robot (normally pitch)
     * ! If you don't want bugs, MAKE SURE THE GYRO IS READY BEFORE CALLING THIS
     * METHOD !
     * 
     * @return The angle of the robot in degrees (-180 to 180).
     */
    public double getVAngle() {
        if (!isReady())
            throw new IllegalStateException("Gyro is not ready!");
        return ((useRoll) ? gyroscopeAHRS.getRoll() : gyroscopeAHRS.getPitch()) * (upsideDown ? -1 : 1);
    }

    /**
     * Get the YAW of the robot
     * ! If you don't want bugs, MAKE SURE THE GYRO IS READY BEFORE CALLING THIS
     * METHOD !
     * 
     * @return The YAW angle of the robot in degrees (-180 to 180).
     */
    public double getYaw() {
        if (!isReady())
            throw new IllegalStateException("Gyro is not ready!");
        return gyroscopeAHRS.getYaw();
    }

    /**
     * Set the current yaw value to be the new zero value.
     */
    public void zeroYaw() {
        if (!isReady())
            throw new IllegalStateException("Gyro is not ready!");
        gyroscopeAHRS.zeroYaw();
    }

    @Override
    public void periodic() {
        if (!isReady())
            return;
        SmartDashboard.putNumber("Gyro Vertical Angle (Pitch)", getVAngle());
        SmartDashboard.putNumber("Yaw", gyroscopeAHRS.getYaw());
        SmartDashboard.putNumber("Pitch", gyroscopeAHRS.getPitch());
        SmartDashboard.putNumber("Roll", gyroscopeAHRS.getRoll());
    }
}
