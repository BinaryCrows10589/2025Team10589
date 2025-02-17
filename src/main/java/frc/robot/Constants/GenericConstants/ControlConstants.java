package frc.robot.Constants.GenericConstants;

// Do not create an instants of a constant class

public final class ControlConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kButtonBoardPort = 1;
    public static final int kButtonBoardAltPort = 2;
    public static final double kDeadband = .07;
    public static boolean slowModeActive = false;
    public static final double kTranslationXSlowModeMultipler = .2;
    public static final double kTranslationYSlowModeMultipler = .2;
    public static final double kRotationSlowModeMultipler = .2;

    public static final int[] kElevatorInPositionColor = {0, 255, 200}; // Greenish Cyan
    public static final int[] kElevatorInMotionColor = {255, 255, 0}; // Yellow
    public static final int[] kCoralIntakingColor = {255, 0, 0}; // Red
    public static final int[] kCoralIntakedColor = {0, 255, 0}; // Green
    public static final int[] kCoralOuttakedColor = {255, 0, 255}; // Purple
    public static final int[] kAlgaeOuttakeReady = {255, 200, 0}; // Orange
    public static final int[] kClimberInMotionColor = {200, 255, 255}; // Baby Blue
    public static final int[] kClimberInPositionColor = {255, 200, 200}; // Pink

    public static final int kButtonBoardAutoPositioningPort = 1;
    public static final int kButtonBoardNormalPort = 3; // TODO: Change this?
    

    public static boolean kIsDriverControlled = true;
    public static boolean axisLockMode = false;
}
