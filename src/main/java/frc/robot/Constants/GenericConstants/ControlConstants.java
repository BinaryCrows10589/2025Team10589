package frc.robot.Constants.GenericConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

// Do not create an instants of a constant class

public final class ControlConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kButtonBoardNormalPort = 1; // TODO: Change this?
    public static final int kButtonBoardAutoPositioningPort = 2;
    public static final int kButtonBoardAltPort = 3;
    public static final double kDeadband = .07;
    public static boolean slowModeActive = false;
    public static final double kTranslationXSlowModeMultipler = .2;
    public static final double kTranslationYSlowModeMultipler = .2;
    public static final double kRotationSlowModeMultipler = .2;

    public static final int[] kElevatorInBasement = {50, 200, 150}; // Greenish Cyan
    public static final int[] kElevatorInBasementWithCoral = {0, 255, 0}; // Greenish 
    public static final int[] kElevatorAtL2 = {255, 255, 0}; // Red
    public static final int[] kElevatorAtL3 = {255, 255, 255}; // Red
    public static final int[] kElevatorAtL4 = {255, 0, 0}; // Red



    //public static final int[] kCoralIntakingColor = {255, 0, 0}; // Red
    public static final int[] kCoralIntakedColor = {0, 255, 0}; // Green
    public static final int[] kCoralOuttakedColor = {255, 0, 255}; // Purple
    public static final int[] kAutoPositionColor = {220, 80, 10}; // Orange
    public static boolean kHasCoral = false;
    //public static final int[] kClimberInMotionColor = {200, 255, 255}; // Baby Blue
    //public static final int[] kClimberInPositionColor = {255, 200, 200}; // Pink
    public static boolean kIsDriverControlled = true;
    public static boolean axisLockMode = false;
    public static boolean isScrolling = false;
    public static AutonPoint robotStartPosition = new AutonPoint(new Pose2d());
    public static Pose2d robotStartPositionTolorence = new Pose2d(.025, .025, Rotation2d.fromDegrees(2));
    public static double desiredElevatorPosition = ElevatorSubsystem.resolveElevatorPosition(ElevatorSubsystem.ElevatorPosition.L4);

}
