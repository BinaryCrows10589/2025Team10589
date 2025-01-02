package frc.robot.Constants;

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

    public static boolean kIsDriverControlled = true;
}
