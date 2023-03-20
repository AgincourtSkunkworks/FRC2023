// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


public class Robot extends TimedRobot {
    // Initialize objects
    TalonFX rightMotor1 = new TalonFX(14);
    TalonFX rightMotor2 = new TalonFX(15);
    TalonFX leftMotor1 = new TalonFX(2);
    TalonFX leftMotor2 = new TalonFX(3);
    TalonFX armMotor = new TalonFX(0);
    
    TalonFX[] leftMotors = {leftMotor1, leftMotor2};
    TalonFX[] rightMotors = {rightMotor1, rightMotor2};
    TalonFX[] motors = {leftMotor1, leftMotor2, rightMotor1, rightMotor2};

    Joystick controller = new Joystick(0);
    AHRS gyroscopeAhrs = new AHRS(SPI.Port.kMXP);

    // Configuration Variables
    final int initialState = 0; // Initial state when autonomous is enabled (used for debugging usually)
    final int autonomousMode = 0; // Mode to use for autonomous docking, 0 = Mini Bang Bang w/ Constant, 1 = PID
    final int turnRadius = 90; // Amount to turn when trying to do a 90 degree turn. Generally to account for drift.
    final double teleopMoveScale = 0.7; // Percent to scale the controller input by when moving (forward or backward)
    final double teleopTurnScale = 0.5; // Percent to scale the controller input by when turning (controllers aren't in same direction [0 is considered no direction])
    final double leftMotorSpeedOffset = 0; // Percent offset (0-1) for left motor speed (to ensure that it can drive straight)
    final double rightMotorSpeedOffset = 0; // Percent offset (0-1) for right motor speed (to ensure that it can drive straight)
    final double onFloorMin = -3; // Pitch degrees to be considered on floor
    final double onFloorMax = 3; // Pitch degrees to be considered on floor
    final double timeToNonCommunity = 3000; // Time in milliseconds to drive from spawn point out of the community, for dumb autonomous
    final double armTurnSpeed = -0.2; // Speed to turn the arm at when turning the arm
    final double armManualOverrideSpeed = -0.2; // Speed to turn the arm at when manually overriding the arm
    final double autonomousMoveSpeed = 0.2; // Speed to move at normally while in automous
    final double autonomousDockSpeed = 0.3; // Speed to move forward while attempting to dock
    final double autonomousBangBangConstant = 0.038; // Constant to multiply speed by when using autonomous state 0 (mini bang bang)
    final long autonomousFloorCheckInterval = 100; // Interval to check gryo at to determine if we're at the docking station (in milliseconds)
    final long autonomousDockCheckInterval = 0; // Interval to check gyro at while attempting to dock (in milliseconds)
    final long armMaintainCheckInterval = 750; // Interval to check arm position while maintaining the middle arm position (in milliseconds)
    final long debugOutputPrintInterval = 500; // Interval to print debug output (in milliseconds)
    final boolean useRoll = true; // Whether to use roll instead of pitch for pitch related operations
    final boolean upsideDownGyro = false; // Whether the gyro is upside down vertically (will reverse gyro logic for pitch)
    final boolean debugMode = false; // Debug mode is used to print certain values used for debugging purposes.

    // Runtime Variables
    int state, startPos, armPos;
    double pitchDegrees, armMaintainSpeed;
    long lastRunTime, lastDebugOutputTime, lastArmCheckTime, dockingStartTime;
    boolean waiting, armTransition, lastUpper, offsetOverride;

    /**
     * Set Left Motor Speeds
     * Note: Left motors are reversed (a negative value will go forward, positive will go backwards)
     * 
     * @param speed Percent of maximum motor speed (1 being max)
     */
    private void setLeftMotorSpeed(double speed) {
        for (TalonFX motor : leftMotors)
            motor.set(ControlMode.PercentOutput, speed + speed * leftMotorSpeedOffset);
    }

    /**
     * Set Right Motor Speeds
     * Note: Right motors are proper (a positive value will go forward, negative will go backwards)
     * 
     * @param speed Percent of maximum motor speed (1 being max)
     */
    private void setRightMotorSpeed(double speed) {
        for (TalonFX motor : rightMotors)
            motor.set(ControlMode.PercentOutput, speed + speed * rightMotorSpeedOffset);
    }

    /**
     * Set Arm Motor Speed
     * @param speed Percent of maximum motor speed (1 being max)
     */
    private void setArmMotorSpeed(double speed) {
        armMotor.set(speed);
    }

    /**
     * Set All Motor Speeds w/ Motor Direction Correction
     * @param speed Percent of maximum motor speed (1 being max)
     */
    private void setMotorSpeedCorrected(double speed) {
        setLeftMotorSpeed(speed);
        setRightMotorSpeed(-speed);
    }

    /**
     * Turn the robot automatically. Call this function periodically while trying to turn.
     * @param speed Speed to turn at.
     * @param degrees Amount of degrees to turn, not 100% precise. Do not do more than 180 degree turns (this isn't made for it -- at least not yet)
     * @param initial Whether this is the first time the function is called
     * @return True if turning is complete, false otherwise
     */
    private boolean turnRobot(double speed, int degrees, boolean initial) {
        if (initial) {
            setMotorSpeedCorrected(0); // Stop moving, to ensure proper zeroYaw() placement
            gyroscopeAhrs.zeroYaw(); // Reset YAW to current face
            if (degrees < 0) { // Left turn
                setLeftMotorSpeed(-speed);
                setRightMotorSpeed(-speed);
            } else if (degrees > 0) { // Right turn
                setLeftMotorSpeed(speed);
                setRightMotorSpeed(speed);
            }
        }
        
        return (degrees < 0) ? degrees - gyroscopeAhrs.getYaw() >= 0 : gyroscopeAhrs.getYaw() - degrees >= 0;
    }

    /**
     * Turn the robot arm automatically. Call this function periodically while trying to turn. The arm motor came straight from hell.
     * @param speed Speed to turn at.
     * @param targetPosition Position to move the arm to
     * @param initial Whether this is the first time the function is called
     * @return True if turning is complete, false otherwise
     */
    private boolean turnArm(double speed, int targetPosition, boolean initial) {
        if (targetPosition == 1) { // Middle
            setArmMotorSpeed(speed);
            return true;
        } else { // Low
            setArmMotorSpeed(0);
            return true;
        }
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        for (TalonFX motor : motors)
            motor.setNeutralMode(NeutralMode.Brake);
        CameraServer.startAutomaticCapture(); // Start the webcam
    }

    @Override
    public void robotPeriodic() {
        // ! FIXME: SmartDashboard numbers are just overlaying each other, instead of replacing like it should
        SmartDashboard.putNumber("Left Motor 1 Pos", leftMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Motor 2 Pos", leftMotor2.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Motor 1 Pos", rightMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Motor 2 Pos", rightMotor2.getSelectedSensorPosition());

        if (gyroscopeAhrs.isConnected()) {
            SmartDashboard.putNumber("Yaw", gyroscopeAhrs.getYaw());
            SmartDashboard.putNumber("Pitch", gyroscopeAhrs.getPitch());
            SmartDashboard.putNumber("Roll", gyroscopeAhrs.getRoll());
        }
    }

    @Override
    public void autonomousInit() {
        /* 
         * State (used to determine operations in periodic)
         * 0 - Initialization
         * 1 - Moving to Charging
         * 2 - Charging Station Docking
         * 3 - Done Docking (in theory)
         * -1 - Error State
         * -2 - "Dumb" Autonomous (move forward out of community and gain a few points)
         */
        state = initialState;
        lastRunTime = 0;
        waiting = false;
        if (debugMode)
            System.out.println("Autonomous Initialization Complete");
    }

    @Override
    @SuppressWarnings("all") // I'm not wrong, you're wrong. Get rid of squiggly lines from config variables ("dead code", "unused code", "redundant check")
    public void autonomousPeriodic() {
        if (!gyroscopeAhrs.isConnected()) { // If gyro is not connected, return
            System.out.println("[AUTONOMOUS] ERROR: Gyro is not connected!");
            return;
        }
        if (debugMode && System.currentTimeMillis() - lastDebugOutputTime >= debugOutputPrintInterval) {
            System.out.printf("Current Autonomous State: %d%n", state);
            lastDebugOutputTime = System.currentTimeMillis();
        }

        long curTime = System.currentTimeMillis();

        // READ THIS vv
        // WARNING: Using a switch statement here causes the god of all confusing bugs to appear. Literally no idea why.
        // READ THIS ^^
        if (state == -2) { // "Dumb" Autonomous (Budget Points)
            if (lastRunTime == 0) { // Initial
                setMotorSpeedCorrected(autonomousMoveSpeed); 
                lastRunTime = curTime;
            }
            else if (curTime - lastRunTime >= timeToNonCommunity) {
                setMotorSpeedCorrected(0);
                state = -1; // Error State
            }
        } else if (state == 0) { // Initialization
            gyroscopeAhrs.zeroYaw();
            state = 1; // Done Initializing
        } else if (state == 1) { // Moving Forward to Charging Station
            if (!waiting) {
                setMotorSpeedCorrected(autonomousMoveSpeed);
                waiting = true;
            }
            if (curTime - lastRunTime >= autonomousFloorCheckInterval) {
                double pitchDegrees = (useRoll) ? gyroscopeAhrs.getRoll() : gyroscopeAhrs.getPitch();
                lastRunTime = curTime;
                if (onFloorMin > pitchDegrees || pitchDegrees > onFloorMax) {
                    if (debugMode) System.out.printf("Pitch OUT of floor range (%f out of %f-%f)%n", pitchDegrees, onFloorMin, onFloorMax);
                    setMotorSpeedCorrected(0);
                    state = 2; // Dock with Charging Station
                    waiting = false;
                    lastRunTime = 0;
                } else if (debugMode) System.out.printf("Pitch IN floor range (%f <= %f <= %f)%n", onFloorMin, pitchDegrees, onFloorMax);
            }
        } else if (state == 2) { // Dock with Charging Station
            if (!waiting) {
                setMotorSpeedCorrected(autonomousDockSpeed);
                waiting = true;
            }
            if (curTime - lastRunTime >= autonomousDockCheckInterval) {
                double pitchDegrees = (useRoll) ? gyroscopeAhrs.getRoll() : gyroscopeAhrs.getPitch();
                boolean condition = false;
                lastRunTime = curTime;
                if (autonomousMode == 0) { // Mini Bang Bang w/ Constant
                    final double motorSpeed = pitchDegrees * autonomousDockSpeed * autonomousBangBangConstant * (upsideDownGyro ? -1 : 1);
                    setMotorSpeedCorrected(motorSpeed);
                    if (debugMode) System.out.printf("Docking Speed: %f%nPitch: %f%n", motorSpeed, pitchDegrees);
                    // Bang Bang control does not end, even when it's balanced (in case of shift/edge). So the condition is not changed.
                } else if (autonomousMode == 1) { // PID
                    state = -1; // TODO: Implement PID docking control
                }

                if (condition) {
                    setMotorSpeedCorrected(0);
                    state = 3; // Finished docking
                    waiting = false;
                    lastRunTime = 0;
                }
            }
        }
    }

    @Override
    public void teleopInit() {
        if (debugMode) {
            gyroscopeAhrs.zeroYaw();
            System.out.println("Teleoperator Mode Initialized, set gyro to zero Yaw");
        }
        lastDebugOutputTime = 0;

        // 0 = Lowered, 1 = Raised, -1 = Unknown
        armPos = 0;
        armTransition = false;
        offsetOverride = false;
        setMotorSpeedCorrected(0);
    }

    @Override
    @SuppressWarnings("unused") // I'm not wrong, you're wrong. Get rid of squiggly lines from config variables ("dead code", "unused code", "redundant check")
    public void teleopPeriodic() {
        long curTime = System.currentTimeMillis();

        // DRIVE
        double stickLeft = -controller.getRawAxis(1);
        double stickRight = controller.getRawAxis(3);
        if (stickLeft > 0.05 && stickRight < 0.05 || stickLeft < 0.05 && stickRight > 0.05) { // no movement is ~+-0.007, not absolute zero
            setLeftMotorSpeed(stickLeft * ((offsetOverride) ? 1 : teleopMoveScale));
            setRightMotorSpeed(stickRight * ((offsetOverride) ? 1 : teleopMoveScale));
        } else {
            setLeftMotorSpeed(stickLeft * ((offsetOverride) ? 1 : teleopTurnScale));
            setRightMotorSpeed(stickRight * ((offsetOverride) ? 1 : teleopTurnScale));
        }

        // ARM
        // TODO: Redo entire arm control system
        // if (armPos != -1) {
        //     if (armTransition) {
        //         if (turnArm(armTurnSpeed, armPos, false))
        //             armTransition = false;
        //     } else {
        //         // L2 - Low
        //         if (controller.getRawButtonPressed(7) && armPos != 1) {
        //             armTransition = true;
        //             armPos = 0;
        //         }
        //         // L1 - High
        //         else if (controller.getRawButtonPressed(5) && armPos != 2) {
        //             armTransition = true;
        //             armPos = 1;
        //         }
        //         if (armTransition) {
        //             turnArm(armTurnSpeed, armPos, true);
        //         }
        //     }
        // }
        // if (armPos == 1 && !armTransition && curTime - lastArmCheckTime >= armMaintainCheckInterval) // Maintain High
        //     setArmMotorSpeed(armMaintainSpeed);
        // X - Manual Override/Motor Control
        if (controller.getRawButton(1)) {
            // Arm position is no longer known, do not allow automatic movement
            if (debugMode && armPos != -1) System.out.println("Arm Manual Override Activated - Auto Movement Disabled");
            armPos = -1;
            setArmMotorSpeed(armManualOverrideSpeed);
        } else if (armPos == -1)
            setArmMotorSpeed(0);
        // A - Reset Arm Position (also disables manual override) - only use if arm is at the bottom (low)
        if (controller.getRawButtonPressed(2)) {
            setArmMotorSpeed(0);
            armPos = 0;
            if (debugMode) System.out.println("Arm Position Reset - Auto Movement Enabled");
        }
        // R2 - Speed Offset Manual Override
        if (controller.getRawButtonPressed(8)) {
            offsetOverride = true;
            if (debugMode) System.out.println("Speed Offset Manual Override Activated");
        } else if (controller.getRawButtonReleased(8)) {
            offsetOverride = false;
            if (debugMode) System.out.println("Speed Offset Manual Override Deactivated");
        }

        // DEBUG
        if (debugMode && curTime - lastDebugOutputTime >= lastDebugOutputTime) {
            System.out.printf(
                "===%nPitch: %f%nYaw: %f%nRoll: %f%nController Left: %f%nController Right: %f%n",
                gyroscopeAhrs.getPitch(), gyroscopeAhrs.getYaw(), gyroscopeAhrs.getRoll(), stickLeft, stickRight
            );
            lastDebugOutputTime = curTime;
        }
    }


    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
