package frc.robot.Constants.MechanismConstants;

public class OuttakeConstants {
    // speeds or voltages for the wheels
    public static final double kLeftWheel = 0.0;
    public static final double kRightWheel = 0.0;

    public static final int kLeftWheelMotorCANID = 28;
    public static final int kRightWheelMotorCANID = 29;

    public static final int kWheelSmartCurrentLimit = 2; // TODO: MAKE THIS CORRECT OR DEATH OF MOTORS WILL ENSUE

    public static final int kLeftWheelPPIDValue = 0;
    public static final int kLeftWheelIPIDValue = 0;
    public static final int kLeftWheelDPIDValue = 0;

    public static final int kRightWheelPPIDValue = 0;
    public static final int kRightWheelIPIDValue = 0;
    public static final int kRightWheelDPIDValue = 0;
    public static final int kOuttakeCoralSensorStartCANID = 30;
    public static final int kOuttakeCoralSensorEndCANID = 31;
    public static final double kOuttakeCoralSensorMaxCoralDistanceMillimeters = 558.8;
}
