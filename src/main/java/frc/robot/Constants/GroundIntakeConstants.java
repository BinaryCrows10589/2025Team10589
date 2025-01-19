package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GroundIntakeConstants {

    public class PivotContants {
        public static final int kPivotEncoderCANID = 0;
        public static final int kPivotMotorCANID = 0;

        public static final double kRotationOffset = 0;
        public static final double kMaxVoltage = 2;
        public static final double kPivotGearRatio = 121.5/1;

        public static final double kPivotPPIDValue = 0;
        public static final double kPivotIPIDValue = 0;
        public static final double kPivotDPIDValue = 0;
        public static final double kPivotAngleToloranceRotations = Units.degreesToRotations(3);
    }

    public class IntakeWheelsConstants {
        public static final int kIntakeWheelsMotorCANID = 0;
    }
    
    public class CoralSensorIntakeConstants {
        public static final int CoralSensorIntakeID = 0;
        public static final double kCoralIntakeSensorMaxCoralDistanceMilameters = 330.2;
    }
    

   

    

}
