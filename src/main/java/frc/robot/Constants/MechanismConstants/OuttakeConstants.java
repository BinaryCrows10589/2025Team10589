package frc.robot.Constants.MechanismConstants;

public class OuttakeConstants {
    // speeds or voltages for the wheels
    public static final double kLeftWheelOuttake = 12;
    
    public static final double kRightWheelOuttake = 9;
    public static final double kRightWheelOuttakeL4 = 11;//9;
    public static final double kRightWheelOuttakeL3 = 5;//3.2;
    public static final double kRightWheelOuttakeL2 = 4;//3;


    public static final double kLeftWheelL1 = 0.0;
    public static final double kRightWheelL1 = 0.0;

    public static final int kLeftWheelMotorCANID = 28;
    public static final int kRightWheelMotorCANID = 29;

    public static final int kWheelSmartCurrentLimit = 12;
    public static final double kMaxWheelVoltage = 12;


    public static final double kHoldCoralVoltage = 1;

    public static final int kRightWheelPPIDValue = 0;
    public static final int kRightWheelIPIDValue = 0;
    public static final int kRightWheelDPIDValue = 0;

    public static final int kOuttakeCoralSensorStartCANID = 30;
    public static final int kOuttakeCoralSensorEndCANID = 31;
    public static final double kOuttakeCoralSensorMaxCoralDistanceMillimeters = 40;
    public static final double kRightWheelIntake = -4.0;

}
