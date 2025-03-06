package frc.robot.Utils.GeneralUtils;

public class PercentError {
    public static double getPercentError(double actualValue, double expectedValue) {
        return (actualValue - expectedValue) / expectedValue;
    }
}
