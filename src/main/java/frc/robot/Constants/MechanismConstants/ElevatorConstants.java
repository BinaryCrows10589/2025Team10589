package frc.robot.Constants.MechanismConstants;

public class ElevatorConstants {
    public static final double kDefaultElevatorPosition = 0.0;

    public static final int kElevatorMasterMotorCANID = 24; // Left motor is master
    public static final int kElevatorSlaveMotorCANID = 25; // Right motor is slave
    public static final int kElevatorEncoderCANID = 26;

    public static final double kElevatorGearRatio = 8.25;

    public static final double kMaxVoltage = 12;

    public static final double kElevatorPPIDValue = 50;
    public static final double kElevatorIPIDValue = 0;
    public static final double kElevatorDPIDValue = 0;
    public static final double kElevatorGPIDValue = .85;

    public static final double kElevatorSPIDValue = 0;
    public static final double kElevatorVPIDValue = 0;
    public static final double kElevatorAPIDValue = 0;

    public static final double kMotionMagicCruiseVelocity = 0;
    public static final double kMotionMagicAcceleration = 0;
    public static final double kMotionMagicJerk = 0;
    
    public static final boolean isSlaveReversed = false;

    public static final double kGroundIntakePassoffTolorence = 0.02;
    public static final double kElevatorScorePositionTolorence = 0.02;
    public static final double kElevatorScoreBargePositionTolorence = 0.02;
    public static final double kElevatorScoreProcessorPositionTolorence = 0.02;
    public static final double kElevatorFunnelTolorence = 0.02;

    public static final double kForwardSoftLimit = .8643; // ???

    public static final double kReverseSoftLimit = 0.0;

    public static final double kManualMovementUpSpeed = 0.03583333333 / 2; // Revelutions Per Frame

    public static final double kManualMovementDownSpeed = -0.03583333333 / 2; // Revelutions Per Frame

    public static final double kElevatorEncoderOffset = .029;

    public static final double kElevatorOuttakeTolerance = 1;

    public static final double kElevatorIntakeTolerance = 2;

    public static final double kBasementShutoffTolerance = 0.02;

    public static final double kInPositionTolorence = 0.02;

    public static final double kCatchTolorence = 0.08;


}
