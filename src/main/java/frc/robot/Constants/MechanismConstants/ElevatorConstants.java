package frc.robot.Constants.MechanismConstants;

public class ElevatorConstants {
    public static final double kDefaultElevatorPosition = 0.0;

    public static final int kElevatorMasterMotorCANID = 24; // Left motor is master
    public static final int kElevatorSlaveMotorCANID = 25; // Right motor is slave
    public static final int kElevatorEncoderCANID = 26;

    public static final double kElevatorGearRatio = 8.25;

    public static final double kMaxVoltage = 12;

    public static final double kElevatorPPIDValue = 0;
    public static final double kElevatorIPIDValue = 0;
    public static final double kElevatorDPIDValue = 0;
    public static final double kElevatorGPIDValue = .6;

    public static final double kElevatorSPIDValue = 0;

    public static final double kMotionMagicCruiseVelocity = 0;
    public static final double kMotionMagicAcceleration = 0;
    public static final double kMotionMagicJerk = 0;
    public static final boolean isSlaveReversed = false;

    public static final double kGroundIntakePassoffTolorence = 0.00833333333;
    public static final double kElevatorScorePositionTolorence = 0.00833333333;
    public static final double kElevatorScoreBargePositionTolorence = 0.00833333333;
    public static final double kElevatorScoreProcessorPositionTolorence = 0.00833333333;
    public static final double kElevatorFunnelTolorence = 0.00833333333;

    public static final double kForwardSoftLimit = .9; // ???

    public static final double kReverseSoftLimit = 0.0;

    public static final double kManualMovementUpSpeed = 0.005;

    public static final double kManualMovementDownSpeed = -0.005;

    public static final double kElevatorEncoderOffset = .022;

    public static final double kElevatorOuttakeTolerance = 1;

    public static final double kElevatorIntakeTolerance = 2;


}
