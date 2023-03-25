// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
public static enum AutonomousSequence { // Enum to use for different autonomous sequences
    LEAVE, // Leave the community area
    DOCK, // Dock with the charging station
    LEAVE_DOCK, // Leave the community area, then dock
    ARM_LEAVE_DOCK, // Raise the arm, leave the community area, then dock. NOTE: Only designed for PID arm movement
    NONE
}

public static enum ArmMovement { // Enum to use for types of arm movement
    BANG_BANG,
    PID
}

public final static class Buttons { // Button Name -> ID Mapping
    public static final int A = 2;
    public static final int B = 3;
    public static final int X = 1;
    public static final int Y = 4;
    public static final int BACK = 9;
    public static final int START = 10;
    public static final int L1 = 5; // ! Currently broken on the controller (not the code)
    public static final int L2 = 7;
    public static final int R1 = 6;
    public static final int R2 = 8;
}

public final static class Joystick { // Joystick Name -> ID Mapping
    public static final int LX = 0;
    public static final int LY = 1;
    public static final int RX = 2;
    public static final int RY = 3;
}

public final static class ID { // Motor Name -> ID Mapping
    public static final int L1 = 14; // Left Motor 1 ID
    public static final int L2 = 15; // Left Motor 2 ID
    public static final int R1 = 12; // Right Motor 1 ID
    public static final int R2 = 13; // Right Motor 2 ID
    public static final int ARM = 11; // Arm Motor ID
    public static final int JOYSTICK = 0; // Joystick ID
}

public final static class Arm {
    final static int OVERRIDE_BTN = Buttons.X; // Button to use for manual override
    final static int OVERRIDE_REVERSE_BTN = Buttons.A; // Button to use for manual override reverse
    final static int RESET_BTN = Buttons.Y; // Button to use for pos offset reset
    final static int PS_LOW_BTN = Buttons.L2; // Button to use for low preset
    final static int PS_HIGH_BTN = Buttons.R2; // Button to use for high preset
    final static double TOLERANCE = 250; // Tolerance for arm position (in encoder ticks) for Bang Bang control
    final static double LIMIT = 19000; // Software limit for arm position to hard stop at (in encoder ticks)
    final static double SPEED = 0.18; // Speed for arm movement, when using bang bang
    final static double REVERSE_SPEED = 0; // Speed for arm movement in reverse, when using bang bang (0 to use brake)
    final static double OVERRIDE_SPEED = 0.25; // Speed for arm movement when using manual override
    final static double OVERRIDE_REVERSE_SPEED = -0.15; // Speed for arm movement when using manual override in reverse
    final static double PS_LOW_POS = 0; // Low position for arm (in encoder ticks)
    final static double PS_HIGH_POS = 18500; // High position for arm (in encoder ticks)
    final static ArmMovement TYPE = ArmMovement.PID; // Whether to use bang bang or PID for arm movement
    public final class PID {
        final static double P = 0.000035; // Proportional constant for PID
        final static double P_D = 0; // Proportional constant for PID (when moving down)
        final static double I = 0; // Integral constant for PID
        final static double D = 0.0000005; // Derivative constant for PID
        final static double I_TOLERANCE = 0; // Tolerance for integral constant for PID (0 to disable)
    }
}

public final static class Drive {
    final static double BRAKE_THRESHOLD = 0.005; // Speed threshold to round to 0 (and thus brake)
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
    final static AutonomousSequence SEQUENCE = AutonomousSequence.DOCK; // Sequence to use in autonomous
    final static int TURN_RADIUS = 90; // Amount to turn (in degrees) for turning commands
    final static double MOVE_SPEED = 0.24; // Percent speed (0-1) for moving the robot
    final static double TURN_SPEED = 0.3; // Percent speed (0-1) for turning the robot
    final static double FLOOR_ANGLE = 3; // Absolute angle (in degrees) to be considered on the floor
    final static double COMM_LEAVE_TIME = 0.6; // Time in seconds that are needed to leave the community area (after passing charging station)
    final static double DOWN_SCALE = 0.85; // Amount to scale the speed of the drive when going down the charging station (for LEAVE_DOCK)
    public final class DockPID {
        final static double P = 0.0125; // Proportional constant for PID
        final static double I = 0.0045; // Integral constant for PID
        final static double D = 0.0008; // Derivative constant for PID
        final static double I_TOLERANCE = 1; // Tolerance for integral constant for PID (0 to disable)
        final static double P_REVERSE_SCALE = 1.05; // Amount to scale the speed of the drive when going up the charging station in reverse (for LEAVE_DOCK)
    }
}

public final static class TeleOp {
    final static int LEFT_DRIVE_STICK = Joystick.LY; // Joystick to use for left motor control
    final static int RIGHT_DRIVE_STICK = Joystick.RY; // JOystick to use for right motor control
    final static int TURBO_BTN = Buttons.R1; // Button to use to override drive speed scaling
    final static double MOVE_SCALE = 0.7; // Percent (0-1) to scale speed when moving the robot
    final static double TURN_SCALE = 0.5; // Percent (0-1) to scale speed when turning the robot
}
}
