// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Compressor;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


public class Robot extends TimedRobot {
    // Initialize objects
    TalonSRX rightMotor1 = new TalonSRX(8);
    TalonSRX rightMotor2 = new TalonSRX(7);
    TalonSRX rightMotor3 = new TalonSRX(6);
    TalonSRX leftMotor1 = new TalonSRX(0);
    TalonSRX leftMotor2 = new TalonSRX(1);
    TalonSRX leftMotor3 = new TalonSRX(2);

    TalonSRX shooterMotor1 = new TalonSRX(4);
    TalonSRX shooterMotor2 = new TalonSRX(3);

    
    TalonSRX[] leftMotors = {leftMotor1, leftMotor2, leftMotor3};
    TalonSRX[] rightMotors = {rightMotor1, rightMotor2, rightMotor3};
    TalonSRX[] shooterMotors = {shooterMotor1, shooterMotor2};
    TalonSRX[] motors = {leftMotor1, leftMotor2, leftMotor3, rightMotor1, rightMotor2, rightMotor3, shooterMotor1, shooterMotor2};

    Joystick joystick = new Joystick(0);

    AHRS ahrs = new AHRS(SPI.Port.kMXP);

    // Configuraton Variables
    final int startPosOverride = -2; // -2 = None; -1 = Left; 0 = Center; 1 = Right
    final float onFloorMin = -3; // Pitch degrees to be considered on floor
    final float onFloorMax = 3; // Pitch degrees to be considered on floor
    final float dockedMin = -3; // Pitch degrees to be considered docked (minimum range)
    final float dockedMax = 3; // Pitch degrees to be considered docked (maximum range)
    final double autonomousMoveSpeed = 0.5; // Speed to move at normally while in automous
    final double autonomousTurnSpeed = 0.3; // Speed to turn at while in autonomous mode
    final double autonomousDockSpeed = 0.1; // Speed to move forward while attempting to dock
    final long autonomousFloorCheckInterval = 100; // Interval to check gryo at to determine if we're at the docking station
    final long autonomousDockCheckInterval = 100; // Interval to check gyro at while attempting to dock

    // Runtime Variables
    int state, startPos;
    double pitchDegrees;

    // This code works (initializes compressor and turns it on), however pressure is being lost in 
    // the physical system making it unable to hold pressure, which means we can't continue testing.
    // Compressor pcmCompressor = new Compressor(2, PneumaticsModuleType.CTREPCM);
    

    /**
     * Set Left Motor Speeds
     * 
     * @param speed Percent of maximum motor speed (1 being max)
     */
    private void setLeftMotorSpeed(double speed) {
        for (TalonSRX motor : leftMotors)
            motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Set Right Motor Speeds
     * 
     * @param speed Percent of maximum motor speed (1 being max)
     */
    private void setRightMotorSpeed(double speed) {
        for (TalonSRX motor : rightMotors)
            motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Set All Motor Speeds
     * 
     * @param speed Percent of maximum motor speed (1 being max)
     */
    private void setMotorSpeed(double speed) {
        setLeftMotorSpeed(speed);
        setRightMotorSpeed(speed);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        for (TalonSRX motor : motors) {
            motor.configPeakCurrentLimit(45); // once at 60 (40 previously) amps, limit current
            motor.configPeakCurrentDuration(80); // for 80ms
            motor.configContinuousCurrentLimit(25); // set to 35 (20 previously) amps which will make robot slower
            motor.enableCurrentLimit(true); // enable
        }
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {
        /* 
         * State (used to determine operations in periodic)
         * 0 - Initialization
         * 1 - Post-Initialization (Unmoved)
         * 2 - Turning (if needed)
         * 3 - Moving to Charging
         * 4 - Charging Station Docking
         * 5 - Done Docking (in theory)
         * -1 - Error State
         */
        state = 0;
    }

    @Override
    public void autonomousPeriodic() {
        if (!ahrs.isConnected()) return; // If gyro is not connected, return

        switch (state) {
            case 0: // Initialization
                ahrs.zeroYaw();
                state = 1;
            case 1: // Post-Initialization
                if (startPosOverride != -2)
                    startPos = startPosOverride;
                else {
                    // TODO: Implement left/center/right starting position detection
                }
                state = (startPos != 0) ? 2 : 3; // Reminder: Only set state when startPos has been successfully determined
            case 2: // Turning (if needed)
                switch (startPos) {
                    case -1: // Left
                        ;
                    case 1: // Right
                        ;
                }
                // Reminder: Only set state when movement for 2 has been completed
                // Can probably use gyro to figure out once docking station is reached. Timings/turns need to be hard coded
                state = 3;
            case 3: // Moving Forward to Charging Station
                pitchDegrees = ahrs.getPitch();
                setMotorSpeed(autonomousMoveSpeed);
                while (!(onFloorMin <= pitchDegrees && pitchDegrees <= onFloorMax)) {
                    try {
                        wait(autonomousFloorCheckInterval);
                    } catch (InterruptedException e) {
                        state = -1;
                        return;
                    }
                    pitchDegrees = ahrs.getPitch();
                }
                state = 4;
            case 4: // Dock with Charging Station
                pitchDegrees = ahrs.getPitch();
                setMotorSpeed(autonomousDockSpeed);
                while (!(dockedMin <= pitchDegrees && pitchDegrees <= dockedMax)) {
                    try {
                        wait(autonomousDockCheckInterval);
                    } catch (InterruptedException e) {
                        state = -1;
                        return;
                    }
                    pitchDegrees = ahrs.getPitch();
                }
                state = 5; // Finished docking (hopefully?)
        }
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        double stickLeft = joystick.getRawAxis(1);
        double stickRight = -joystick.getRawAxis(3);
        setLeftMotorSpeed(Math.pow(stickLeft, 3));
        setRightMotorSpeed(Math.pow(stickRight, 3));
        // Button 8 = R2
        if (joystick.getRawButton(8)) {
            shooterMotor1.set(ControlMode.PercentOutput, 0.6);
            shooterMotor2.set(ControlMode.PercentOutput, -0.6);

        } else {
            for (TalonSRX motor : shooterMotors)
                motor.set(ControlMode.PercentOutput, 0);
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
