package frc.robot.Utils.GeneralUtils;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class Tolerance {
    public static boolean inTolorance(double currentValue, double desiredValue, double tolerance) {
        return Math.abs(currentValue - desiredValue) <= tolerance;
    }

    public static boolean inToloranceRotation(Rotation2d currentValue, Rotation2d desiredValue, double toleranceRad) {
        return Math.abs(currentValue.minus(desiredValue).getRadians()) <= toleranceRad;
    }

    public static boolean[] inTolerancePose2d(Pose2d currentValue, Pose2d desiredValue, Pose2d tolerance) {
        return new boolean[] {
            inTolorance(currentValue.getX(), desiredValue.getX(), tolerance.getX()),
            inTolorance(currentValue.getY(), desiredValue.getY(), tolerance.getY()),
            inToloranceRotation(currentValue.getRotation(), desiredValue.getRotation(), tolerance.getRotation().getRadians())
        };
    }

    
}
