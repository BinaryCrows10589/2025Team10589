package frc.robot.Utils.GeneralUtils;

public class Tolerance {
    public static boolean inTolorance(double currentValue, double desiredValue, double tolerance) {
        return Math.abs(currentValue - desiredValue) <= tolerance;
    }
}
