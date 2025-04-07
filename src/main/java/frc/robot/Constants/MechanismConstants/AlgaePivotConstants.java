package frc.robot.Constants.MechanismConstants;

import edu.wpi.first.units.measure.Voltage;

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
    public static final double kGroundIntakePositionRotations = .475 * 2/3;
    public static final double kLollypopIntakePositionRotations = .375 * 2/3; 
    public static final double kReefTreeIntakePositionRotations = .475 * 2/3; 
    public static final double kOuttakeProcessorPositionRotations = .367 * 2/3;
    public static final double kOuttakeBargePositionRotations = .2425 * 2/3;
    public static final double kDefultPivotPosition = 0.0;
    public static final double kAlgaePiovtGroundVoltage = 1.0;
    public static final double kAlgaePiovtHoldGroundVoltage = .5;;
}
