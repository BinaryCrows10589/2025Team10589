package frc.robot.Constants.MechanismConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GroundIntakeConstants {

    public class PivotContants {
        public static final int kPivotMotorCANID = 18;
        public static final int kPivotEncoderCANID = 19;

        public static final double kRotationOffset = 0;
        public static final double kMaxVoltage = 2;
        public static final double kPivotGearRatio = 121.5/1;

        public static final double kPivotPPIDValue = 0;
        public static final double kPivotIPIDValue = 0;
        public static final double kPivotDPIDValue = 0;
        public static final double kPivotAngleToloranceRotations = Units.degreesToRotations(3);
    }

    public class IntakeWheelsConstants {
        public static final int kIntakeWheelsMotorCANID = 20;
    }
    
    public class IntakeCoralSensorConstants {
        public static final int kIntakeCoralSensorID = 21;
        public static final double kIntakeCoralSensorMaxCoralDistanceMilameters = 330.2;
    }
    

   

    

}
