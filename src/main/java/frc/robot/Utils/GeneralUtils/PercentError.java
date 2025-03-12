package frc.robot.Utils.GeneralUtils;

import edu.wpi.first.math.geometry.Rotation2d;

public class PercentError {
    public static double getPercentError(double actualValue, double expectedValue) {
        return (actualValue - expectedValue) / expectedValue;
    }
    public static double getPercentError(Rotation2d actualValue, Rotation2d expectedValue) {
        return (actualValue.minus(expectedValue).getRadians()) / expectedValue.getRadians();
    }
}
