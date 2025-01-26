package frc.robot.Constants.MechanismConstants;

public class ElevatorConstants {
    public static final double kDefaultElevatorPosition = 0.0;

    public static final int kElevatorMasterMotorCANID = 24; // Left motor is master
    public static final int kElevatorSlaveMotorCANID = 25; // Right motor is slave
    public static final int kElevatorEncoderCANID = 26;

    public static final double kElevatorGearRatio = 6/1;

    public static final double kMaxVoltage = 0;

    public static final double kElevatorPPIDValue = 0;
    public static final double kElevatorIPIDValue = 0;
    public static final double kElevatorDPIDValue = 0;
    public static final double kElevatorGPIDValue = 0;
    public static final double kElevatorSPIDValue = 0;

    public static final double kMotionMagicCruiseVelocity = 0;
    public static final double kMotionMagicAcceleration = 0;
    public static final double kMotionMagicJerk = 0;
    public static final boolean isSlaveReversed = true;

    public static final double kGroundIntakePassoffTolorence = .25;
    public static final double kElevatorScorePositionTolorence = .25;
    public static final double kElevatorScoreBargePositionTolorence = .25;
    public static final double kElevatorScoreProcessorPositionTolorence = .25;
    public static final double kElevatorFunnelTolorence = .25;

    public static final double kForwardSoftLimit = 5; // ???

    public static final double kReverseSoftLimit = 0;

    public static final double kManualMovementUpSpeed = 0.05;

    public static final double kManualMovementDownSpeed = -0.05;

    public static final double kElevatorEncoderOffset = 0;


}
