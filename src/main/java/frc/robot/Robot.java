// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


public class Robot extends TimedRobot {
    // Initialize objects
    TalonFX rightMotor1 = new TalonFX(12);
    TalonFX rightMotor2 = new TalonFX(13);
    TalonFX leftMotor1 = new TalonFX(14);
    TalonFX leftMotor2 = new TalonFX(15);
    CANSparkMax armMotor = new CANSparkMax(7, MotorType.kBrushed);

    Compressor pcmCompressor;
    DoubleSolenoid leftGearBox = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
    DoubleSolenoid rightGearBox = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    
    TalonFX[] leftMotors = {leftMotor1, leftMotor2};
    TalonFX[] rightMotors = {rightMotor1, rightMotor2};
    TalonFX[] motors = {leftMotor1, leftMotor2, rightMotor1, rightMotor2};

    Joystick joystick = new Joystick(0);
    AHRS ahrs = new AHRS(SPI.Port.kMXP);
    AnalogInput ultrasonic = new AnalogInput(0);

    // Configuration Variables
    final int startPosOverride = -2; // -2 = None; -1 = Left; 0 = Center; 1 = Right
    final int initialState = 0; // Initial state when autonomous is enabled (used for debugging usually)
    final int turnRadius = 75; // Amount to turn when trying to do a 90 degree turn. Generally to account for drift.
    final double teleopMoveScale = 0.7; // Percent to scale the controller input by when moving (forward or backward)
    final double teleopTurnScale = 0.5; // Percent to scale the controller input by when turning (controllers aren't in same direction [0 is considered no direction])
    final double leftMotorSpeedOffset = 0; // Percent offset (0-1) for left motor speed (to ensure that it can drive straight)
    final double rightMotorSpeedOffset = 0; // Percent offset (0-1) for right motor speed (to ensure that it can drive straight)
    final double armMotorFullRotationIncrements = 50000; // Number of encoder ticks for a full rotation of the arm motor
    final double onFloorMin = -3; // Pitch degrees to be considered on floor
    final double onFloorMax = 3; // Pitch degrees to be considered on floor
    final double dockedMin = -3; // Pitch degrees to be considered docked (minimum range)
    final double dockedMax = 3; // Pitch degrees to be considered docked (maximum range)
    final double timeToCenter = 4100; // Time in milliseconds to drive from one of the side spawn points to the center
    final double armTurnSpeed = 0.2; // Speed to turn the arm at when turning the arm
    final double armManualOverrideSpeed = 0.2; // Speed to turn the arm at when manually overriding the arm
    final double armMaintainMinSpeed = 0.05; // Minimum speed to maintain middle arm position
    final double armMaintainMaxSpeed = 0.2; // Maximum speed to maintain middle arm position
    final double armMaintainIncrement = 0.01; // Amount to increment the arm speed by when maintaining the middle arm position
    final double autonomousMoveSpeed = 0.4; // Speed to move at normally while in automous
    final double autonomousTurnSpeed = 0.3; // Speed to turn at while in autonomous mode
    final double autonomousDockSpeed = 0.35; // Speed to move forward while attempting to dock
    final long autonomousPostDriveDelay = 500; // Delay (in milliseconds) after driving to the center, so we don't carry energy into the turn and overshoot it.
    final long autonomousTurnCheckInterval = 25; // Interval to check gyro while turning
    final long autonomousFloorCheckInterval = 100; // Interval to check gryo at to determine if we're at the docking station (in milliseconds)
    final long autonomousDockCheckInterval = 0; // Interval to check gyro at while attempting to dock (in milliseconds)
    final long armMaintainCheckInterval = 750; // Interval to check arm position while maintaining the middle arm position (in milliseconds)
    final boolean useRoll = true; // Whether to use roll instead of pitch for pitch related operations
    final boolean debugMode = false; // Debug mode is used to print certain values used for debugging purposes.
    final boolean enableCompressor = false; // Whether to enable the compressor or not

    // Runtime Variables
    int state, startPos, armPos;
    double pitchDegrees, armMaintainSpeed;
    long lastRunTime, lastDebugOutputTime, lastArmCheckTime;
    boolean waiting, armTransition, lastUpper;

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
            ahrs.zeroYaw(); // Reset YAW to current face
            if (degrees < 0) { // Left turn
                setLeftMotorSpeed(-speed);
                setRightMotorSpeed(-speed);
            } else if (degrees > 0) { // Right turn
                setLeftMotorSpeed(speed);
                setRightMotorSpeed(speed);
            }
        }
        
        return (degrees < 0) ? degrees - ahrs.getYaw() >= 0 : ahrs.getYaw() - degrees >= 0;
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
     * Convert the raw value from the ultrasound sensor to distance
     * @param rawValue Raw value from the Analog Input from the ultrasound censor (0 - 4095)
     * @return Distance in centimeters
     */
    private double getUltrasonicDistance(double rawValue) {
        double voltageScaleFactor = 5 / RobotController.getVoltage5V();
        return rawValue * voltageScaleFactor * 0.125;
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        if (enableCompressor)
            pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        leftGearBox.set(kReverse);
        rightGearBox.set(kReverse);
        CameraServer.startAutomaticCapture(); // Start the webcam
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {
        /* 
         * State (used to determine operations in periodic)
         * 0 - Initialization
         * 1 - Post-Initialization (Unmoved)
         * 2 - Turning Stage 1 (turn toward center, perpendicular to docking station)
         * 3 - Turning Stage 2 (driving to center)
         * 4 - Turning Stage 3 (delay after driving to center)
         * 5 - Turning Stage 4 (turn toward docking station)
         * 6 - Moving to Charging
         * 7 - Charging Station Docking
         * 8 - Done Docking (in theory)
         * -1 - Error State
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
        if (!ahrs.isConnected()) return; // If gyro is not connected, return
        if (debugMode) System.out.printf("Current Autonomous State: %d%n", state);

        long curTime = System.currentTimeMillis();

        // READ THIS vv
        // WARNING: Using a switch statement here causes the god of all confusing bugs to appear. Literally no idea why.
        // READ THIS ^^
        if (state == 0) { // Initialization
            ahrs.zeroYaw();
            state = 1; // Done Initializing
        } else if (state == 1) { // Post-Initialization
            // double ultrasoundReading = getUltrasonicDistance(ultrasonic.getValue());
            if (startPosOverride != -2)
                startPos = startPosOverride;
            else {
                // TODO: Implement left/center/right starting position detection
            }
            if (startPos < -1 || startPos > 1) state = -1; // Unexpected
            else state = (startPos != 0) ? 2 : 6; // Reminder: Only set state when startPos has been successfully determined
        } else if (state == 2) { // Turning Stage 1 (if needed) -- turn 90 to face center (perpendicular to docking)
            if (curTime - lastRunTime < autonomousTurnCheckInterval) {}
            else if (turnRobot(autonomousTurnSpeed, (startPos == -1) ? turnRadius : -turnRadius, !waiting)) {
                setMotorSpeedCorrected(0);
                waiting = false;
                lastRunTime = 0;
                state = 3; // Turning stage 2
            } else {
                waiting = true;
                lastRunTime = curTime;   
            }
        } else if (state == 3) { // Turning Stage 2 (if needed) -- drive to center
            if (!waiting) {
                setMotorSpeedCorrected(autonomousMoveSpeed);
                waiting = true;
                lastRunTime = curTime;
            }
            if (curTime - lastRunTime >= timeToCenter) {
                setMotorSpeedCorrected(0);
                lastRunTime = 0;
                waiting = false;
                state = 4; // Post-turn wait
            }
        } else if (state == 4) { // Turning Stage 3 (if needed) -- wait a while before turning to face docking station
            if (!waiting) {
                lastRunTime = curTime;
                waiting = true;
            }
            if (curTime - lastRunTime >= autonomousPostDriveDelay) {
                lastRunTime = 0;
                waiting = false;
                state = 5; // Move to docking station
            }
        } else if (state == 5) { // Turning Stage 4 (if needed) -- turn 90 to face docking station
            if (curTime - lastRunTime < autonomousTurnCheckInterval) {}
            else if (turnRobot(autonomousTurnSpeed, (startPos == -1) ? -turnRadius : turnRadius, !waiting)) {
                setMotorSpeedCorrected(0);
                waiting = false;
                lastRunTime = 0;
                state = 6; // Move to docking station
            } else {
                waiting = true;
                lastRunTime = curTime;
            }
        } else if (state == 6) { // Moving Forward to Charging Station
            if (!waiting) {
                setMotorSpeedCorrected(autonomousMoveSpeed);
                waiting = true;
            }
            if (curTime - lastRunTime >= autonomousFloorCheckInterval) {
                double pitchDegrees = (useRoll) ? ahrs.getRoll() : ahrs.getPitch();
                lastRunTime = curTime;
                if (onFloorMin > pitchDegrees || pitchDegrees > onFloorMax) {
                    if (debugMode) System.out.printf("Pitch OUT of floor range (%f out of %f-%f)%n", pitchDegrees, onFloorMin, onFloorMax);
                    setMotorSpeedCorrected(0);
                    state = 7; // Start docking
                    waiting = false;
                    lastRunTime = 0;
                } else if (debugMode) System.out.printf("Pitch IN floor range (%f <= %f <= %f)%n", onFloorMin, pitchDegrees, onFloorMax);
            }
        } else if (state == 7) { // Dock with Charging Station
            if (!waiting) {
                setMotorSpeedCorrected(autonomousDockSpeed);
                waiting = true;
            }
            if (curTime - lastRunTime >= autonomousDockCheckInterval) {
                double pitchDegrees = (useRoll) ? ahrs.getRoll() : ahrs.getPitch();
                lastRunTime = curTime;
                if (dockedMin <= pitchDegrees && pitchDegrees <= dockedMax) {
                    if (debugMode) System.out.printf("Dock IN range (%f <= %f <= %f)%n", dockedMin, pitchDegrees, dockedMax);
                    setMotorSpeedCorrected(0);
                    state = 8; // Finished docking
                    waiting = false;
                    lastRunTime = 0;
                } else if (debugMode) System.out.printf("Dock OUT of range (%f out of %f-%f)%n", pitchDegrees, dockedMin, dockedMax);
            }
        }
    }

    @Override
    public void teleopInit() {
        if (debugMode) {
            ahrs.zeroYaw();
            System.out.println("Teleoperator Mode Initialized, set gyro to zero Yaw");
        }
        lastDebugOutputTime = 0;

        // 0 = Lowered, 1 = Raised, -1 = Unknown
        armPos = 0;
        armTransition = false;
        setMotorSpeedCorrected(0);
    }

    @Override
    @SuppressWarnings("unused") // I'm not wrong, you're wrong. Get rid of squiggly lines from config variables ("dead code", "unused code", "redundant check")
    public void teleopPeriodic() {
        long curTime = System.currentTimeMillis();

        // DRIVE
        double stickLeft = -joystick.getRawAxis(1);
        double stickRight = joystick.getRawAxis(3);
        if (stickLeft > 0.05 && stickRight < 0.05 || stickLeft < 0.05 && stickRight > 0.05) { // no movement is ~+-0.007, not absolute zero
            setLeftMotorSpeed(stickLeft * teleopMoveScale);
            setRightMotorSpeed(stickRight * teleopMoveScale);
        } else {
            setLeftMotorSpeed(stickLeft * teleopTurnScale);
            setRightMotorSpeed(stickRight * teleopTurnScale);
        }

        // ARM
        if (armPos != -1) {
            if (armTransition) {
                if (turnArm(armTurnSpeed, armPos, false))
                    armTransition = false;
            } else {
                // L2 - Low
                if (joystick.getRawButtonPressed(7) && armPos != 1) {
                    armTransition = true;
                    armPos = 0;
                }
                // L1 - High
                else if (joystick.getRawButtonPressed(5) && armPos != 2) {
                    armTransition = true;
                    armPos = 1;
                }
                if (armTransition) {
                    turnArm(armTurnSpeed, armPos, true);
                }
            }
        }
        if (armPos == 1 && !armTransition && curTime - lastArmCheckTime >= armMaintainCheckInterval) // Maintain High
            setArmMotorSpeed(armMaintainSpeed);
        // X - Manual Override/Motor Control
        if (joystick.getRawButton(1)) {
            // Arm position is no longer known, do not allow automatic movement
            if (debugMode && armPos != -1) System.out.println("Arm Manual Override Activated - Auto Movement Disabled");
            armPos = -1;
            setArmMotorSpeed(armManualOverrideSpeed);
        } else if (armPos == -1)
            setArmMotorSpeed(0);
        // A - Reset Arm Position (also disables manual override) - only use if arm is at the bottom (low)
        if (joystick.getRawButtonPressed(2)) {
            setArmMotorSpeed(0);
            armPos = 0;
            if (debugMode) System.out.println("Arm Position Reset - Auto Movement Enabled");
        }

        // GEARBOXES
        // R1 - Both Gearboxes
        if (joystick.getRawButtonPressed(6)) {
            leftGearBox.toggle();
            rightGearBox.toggle();
        }
        // Back - Left Gearbox Manual Override
        if (joystick.getRawButtonPressed(9))
            leftGearBox.toggle();
        // Start - Right Gearbox Manual Override
        if (joystick.getRawButtonPressed(10))
            rightGearBox.toggle();

        // DEBUG
        if (debugMode && curTime - lastDebugOutputTime >= 500) {
            System.out.printf(
                "===%nPitch: %f%nYaw: %f%nRoll: %f%nUltrasonic Raw: %d%nUltrasonic Parsed: %f%nController Left: %f%nController Right: %f%n",
                ahrs.getPitch(), ahrs.getYaw(), ahrs.getRoll(), ultrasonic.getValue(), getUltrasonicDistance(ultrasonic.getValue()), stickLeft, stickRight
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
