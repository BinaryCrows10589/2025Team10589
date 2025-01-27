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

    public static final int[] kElevatorInPositionColor = {0, 255, 200};
    public static final int[] kElevatorInMotionColor = {255, 255, 0};
    public static final int[] kCoralIntakingColor = {255, 0, 0};
    public static final int[] kCoralIntakedColor = {0, 255, 0};
    public static final int[] kCoralOuttakedColor = {255, 0, 255};

    public static boolean kIsDriverControlled = true;
}
