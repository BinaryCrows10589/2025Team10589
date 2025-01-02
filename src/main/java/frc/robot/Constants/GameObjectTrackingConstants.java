package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class GameObjectTrackingConstants {
    public static final Transform3d kExampleGameObjectCameraToRobotCenter =  new Transform3d(0, 0, 0, new Rotation3d());
    public static final Translation2d kGoToObjectPositionTolerance = new Translation2d(1, 1); // TODO: Hone in on these
    public static final double kGameObjectHeight = Units.inchesToMeters(2);
    public static final String kObjectCameraName = "GameObjectCamera";

    public static class RotateToFaceGameObjectConstants {
        public static final double kPRotationConstant = RobotModeConstants.kIsNotSim ? .59 : .59;
        public static final double kIRotationConstant = RobotModeConstants.kIsNotSim ? 0.0 : 0.0;
        public static final double kDRotationConstant = RobotModeConstants.kIsNotSim ? 0.0 : 0.0;
        public static final double kMaxRotationalSpeedInRadsPerSecond = RobotModeConstants.kIsNotSim ? 7.5 : 9.89199998351;
        public static final double kMaxRotationalAccelerationInRadsPerSecond = RobotModeConstants.kIsNotSim ? 9.146 : 2.30100002339;
        public static final TrapezoidProfile.Constraints kRotationPIDControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxRotationalSpeedInRadsPerSecond, kMaxRotationalAccelerationInRadsPerSecond);
        public static final double kRotationTolorence = Units.degreesToRadians(2);
    }
}
