package frc.robot.Constants.MechanismConstants;

public class AlgaePivotConstants {

    public static final double kPivotAngleToleranceRotations = 0.05;
    public static final int kPivotMotorCANID = 32;
    public static final int kPivotEncoderCANID = 38;
    public static final int kSmartCurrentLimit = 20;
    public static final double kPivotPPIDValue = 2.5;//5;
    public static final double kPivotIPIDValue = 0;
    public static final double kPivotDPIDValue = 0;
    public static final double kPivotFPIDValue = 0;;
    public static final double kPivotEncoderOffset = .170;
    public static final double kForwardSoftLimit = .477* 2/3; // .67 .
    public static final double kReverseSoftLimit = .03 * 2/3; //.198
    public static final double kGroundIntakePositionRotations = .47 * 2/3;
    public static final double kLollypopIntakePositionRotations = .375 * 2/3; 
    public static final double kReefTreeIntakePositionRotations = .485 * 2/3; 
    public static final double kOuttakeProcessorPositionRotations = .367 * 2/3;
    public static final double kOuttakeBargePositionRotations = .245 * 2/3;
    public static final double kDefultPivotPosition = 0.0;
}
