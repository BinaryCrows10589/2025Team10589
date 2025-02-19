package frc.robot.Constants.GenericConstants;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;

public class AutoPositionConstants {

    public class AutonScrollConstants {
        public static final double kMaxRotationalSpeedInRadsPerSecond = RobotModeConstants.kIsNotSim ? 10.2 : 9.89199998351;
        public static final double kMaxRotationalAccelerationInRadsPerSecond = RobotModeConstants.kIsNotSim ? 14.2 : 12.928;
        public static final TrapezoidProfile.Constraints kRotationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxRotationalSpeedInRadsPerSecond, kMaxRotationalAccelerationInRadsPerSecond);
        public static final double[] kRotationPIDConstants = {.75, 0, 0};
        public static final double kLockSwervesTime = .1;

    }

    public class ReefPosition1Constants {
        public static final AutonPoint kReefPosition = new AutonPoint(5.250, 5.1518, 60,
            new FudgeFactor(0, 0, 0, 0, 0, 0));
        public static final double[] kTranslationXPIDConstants = {0.75, 0, 0};
        public static final double[] kTranslationYPIDConstants = {.75, 0, 0};
        public static final double[] kRotationPIDConstants = {.9, 0, 0};
        public static final Pose2d kPositionTolorence = new Pose2d(.05, .05, Rotation2d.fromDegrees(3));
        public static final double kMaxTranslationalSpeedInMetersPerSecond = RobotModeConstants.kIsNotSim ? 4.4 : 4.129;//4.4 : 4.129;
        public static final double kMaxTranslationalAccelerationInMetersPerSecond = RobotModeConstants.kIsNotSim ? 4.45 : 5.129;//4.45 : 5.129;
        public static final double kMaxRotationalSpeedInRadsPerSecond = RobotModeConstants.kIsNotSim ? 10.2 : 9.89199998351;
        public static final double kMaxRotationalAccelerationInRadsPerSecond = RobotModeConstants.kIsNotSim ? 14.2 : 12.928;
        public static final TrapezoidProfile.Constraints kRotationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxRotationalSpeedInRadsPerSecond, kMaxRotationalAccelerationInRadsPerSecond);
        public static final double KAngleOfAttack = -60;
        public static final double kMaxTrajectoryTime = 5.0;
        
        public static final double[] kScrollVelocityVector = {4, 0, 0};
        public static final double[] kLockRotationPIDConstants = {0, 0, 0};
        public static final double kMaxScrollTime = 5;
    }
    



    AutonPoint kReefPosition2 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition3 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition4 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition5 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition6 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition7 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition8 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition9 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition10 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition11 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));

    AutonPoint kReefPosition12 = new AutonPoint(0.0, 0.0, 0.0,
        new FudgeFactor(0, 0, 0, 0, 0, 0));
}
