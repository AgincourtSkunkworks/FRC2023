// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
public enum AutonomousSequence {
    LEAVE,
    DOCK,
    LEAVE_DOCK
}

public static final boolean DEBUG_MODE = false; // Debug mode is used to print certain values used for debugging purposes.
public static final double DEBUG_PRINT_INTERVAL = 500; // Interval (in milliseconds) to print debug values

public final static class ID {
    public static final int L1 = 14; // Left Motor 1 ID
    public static final int L2 = 15; // Left Motor 2 ID
    public static final int R1 = 12; // Right Motor 1 ID
    public static final int R2 = 13; // Right Motor 2 ID
    public static final int ARM = 11; // Arm Motor ID
    public static final int JOYSTICK = 0; // Joystick ID
}

public final static class Arm {
    final static double TOLERANCE = 250; // Tolerance for arm position (in encoder ticks)
    final static double LIMIT = 19000; // Software limit for arm position to hard stop at (in encoder ticks)
    final static double SPEED = 0.18; // Speed for arm movement
    final static double OVERRIDE_SPEED = 0.25; // Speed for arm movement when using manual override
    final static double LOW_POS = 0; // Low position for arm (in encoder ticks)
    final static double HIGH_POS = 18400; // Mid position for arm (in encoder ticks)
}

public final static class Drive {
    final static double LM_SPEED_OFFSET = 0; // Percent offset (0-1) for left motor speed (to ensure that it can drive straight)
    final static double RM_SPEED_OFFSET = 0; // Percent offset (0-1) for right motor speed (to ensure that it can drive straight)
    final static boolean LM_INVERSE = false; // Whether the left motors are inverted
    final static boolean RM_INVERSE = true; // Whether the right motors are inverted
}

public final static class Gyro {
    final static boolean USE_ROLL = true; // Whether to use roll instead of pitch for pitch related operations
    final static boolean UPSIDE_DOWN = false; // Whether the gyro is upside down vertically (will reverse gyro logic for pitch)
}

public final static class Autonomous {
    final static AutonomousSequence SEQUENCE = AutonomousSequence.DOCK; // 
    final static int TURN_RADIUS = 90; // Amount to turn (in degrees) for turning commands
    final static double MOVE_SPEED = 0.24; // Percent speed (0-1) for moving the robot
    final static double TURN_SPEED = 0.3; // Percent speed (0-1) for turning the robot
    final static double FLOOR_ANGLE = 3; // Absolute angle (in degrees) to be considered on the floor
    final static double COMM_LEAVE_TIME = 3000; // Time in milliseconds that are needed to leave the community area (after passing charging station)
    public final class DockPID {
        // TODO: Tune PID values
        final static double P = 0.0117; // Proportional constant for PID
        final static double I = 0; // Integral constant for PID
        final static double D = 0; // Derivative constant for PID
    }
}

public final static class TeleOp {
    final static double MOVE_SCALE = 0.7; // Percent (0-1) to scale speed when moving the robot
    final static double TURN_SCALE = 0.5; // Percent (0-1) to scale speed when turning the robot
}
}
