package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// Do not create an instants of a constant class

public final class DrivetrainConstants {

    /**
     * Constants for the Swerve Drive
     */
    public static final class SwerveDriveConstants {
        // Driver Interaction constants
        public static final double kMaxSpeedMetersPerSecond = RobotModeConstants.kIsNotSim ? 4.3 : 4.3;
        public static final double kMaxRotationAnglePerSecond = RobotModeConstants.kIsNotSim ? 11.4 : 11.4;

        public static final double kTranslationMaxRateOfChangePerSecond = RobotModeConstants.kIsNotSim ? 8 : 8;
        public static final double kRotationMaxRateOfChangePerSecond = RobotModeConstants.kIsNotSim ? 100 : 100;
        public static final double kDriveFeedForward = 0.04;

        // Name of the CAN Bus the Swerve Drive is on.
        public static final String kCANLoopName = "rio"; // To swich to CANivor set the CANLoopName to the CANivors serial number or name

        // Swerve Module Configuration Constants
        public static final String kFrontLeftModuleName = "FrontLeftModule";
        public static final String kFrontRightModuleName = "FrontRightModule";
        public static final String kBackLeftModuleName = "BackLeftModule";
        public static final String kBackRightModuleName = "BackRightModule";

        // Drive Motor CANIDs
        public static final int kFrontLeftDriveMotorCANID = 5;
        public static final int kFrontRightDriveMotorCANID = 6;
        public static final int kBackLeftDriveMotorCANID = 7;
        public static final int kBackRightDriveMotorCANID = 8;

        public static final boolean kFrontLeftDriveMotorInverted =  RobotModeConstants.kIsNotSim ? false : false;
        public static final boolean kFrontRightDriveMotorInverted = RobotModeConstants.kIsNotSim ? false : false;
        public static final boolean kBackLeftDriveMotorInverted = RobotModeConstants.kIsNotSim ? false : false;
        public static final boolean kBackRightDriveMotorInverted = RobotModeConstants.kIsNotSim ? false : false;

        // Turn Motor CANIDs
        public static final int kFrontLeftTurnMotorCANID = 9;
        public static final int kFrontRightTurnMotorCANID = 10;
        public static final int kBackLeftTurnMotorCANID = 11;
        public static final int kBackRightTurnMotorCANID = 12;

        public static final boolean kFrontLeftTurnMotorInverted = RobotModeConstants.kIsNotSim ? false : true;
        public static final boolean kFrontRightTurnMotorInverted = RobotModeConstants.kIsNotSim ? false : true;
        public static final boolean kBackLeftTurnMotorInverted = RobotModeConstants.kIsNotSim ? false : true;
        public static final boolean kBackRightTurnMotorInverted = RobotModeConstants.kIsNotSim ? false : true;

        // Turning Absolute Encoder CANIDs
        public static final int kFrontLeftTurningAbsoluteEncoderCANID = 13;
        public static final int kFrontRightTurningAbsoluteEncoderCANID = 14;
        public static final int kBackLeftTurningAbsoluteEncoderCANID = 15;
        public static final int kBackRightTurningAbsoluteEncoderCANID = 16;

        // Turning Absolute Encoder Offsets in rotations
        public static final double kFrontLeftTurningAbsoluteEncoderOffsetRotations = 0.458008; //Done
        public static final double kFrontRightTurningAbsoluteEncoderOffsetRotations= 0.963379; //
        public static final double kBackLeftTurningAbsoluteEncoderOffsetRotations = 0.925537;
        public static final double kBackRightTurningAbsoluteEncoderOffsetRotations = 0.749268;
    
        // End of Swerve Module Configuration Constants

        public static final int kGyroCANID = 17; 

        // Kinematic Configuration

        public static final double kDistanceBetweenCentersOfRightAndLeftWheels = Units.inchesToMeters(23.75);
        public static final double kDistanceBetweenCentersOfFrontAndBackWheels = Units.inchesToMeters(23.75);
        public static final double kRadiusFromCenterToFarthestSwerveModule = Math
        .sqrt(((kDistanceBetweenCentersOfRightAndLeftWheels * kDistanceBetweenCentersOfRightAndLeftWheels)
            + (kDistanceBetweenCentersOfFrontAndBackWheels * kDistanceBetweenCentersOfFrontAndBackWheels)));

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2));
        
        public static final String kSwerveDriveModuleStatesLoggerBase = "SwerveDrive/ModuleStates/";
        public static final String kSwerveDriveChassisSpeedLoggerBase = "SwerveDrive/ChassisSpeeds/";
        public static final String kSwerveDriveDesiredChassisSpeedLoggerBase = "SwerveDrive/DesiredChassisSpeeds/";
        public static final double kVoltageForMaxSpeed = 12;
        public static final double kVoltageFeedforward = 0.1;
    }

    /**
     * Constants for the Swerve Modules
     */
    public static final class SwerveModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * 2 * Math.PI;

        // Motor Control Configuration Values
        public static final double kPModuleDrivePIDValue = 0.0001;
        public static final double kIModuleDrivePIDValue = 0.00000125;
        public static final double kDModuleDrivePIDValue = 0;
        public static final double kFFModuleDrivePIDValue = 0;
        public static final double kIZoneModuleDrivePIDValue = 0.0;

        public static final double kPModuleSIMDrivePIDValue = .1;
        public static final double kIModuleSIMDrivePIDValue = 2;
        public static final double kDModuleSIMDrivePIDValue = 0;

        // If useing TalonFX Swerve these can not be set. Instead please rely on the the max and min voltage.
        public static final double kDriveMotorMinPercentOutput = -1;
        public static final double kDriveMotorMaxPercentOutput = 1;

        public static final double kPModuleTurnPIDValue = .25;
        public static final double kIModuleTurnPIDValue = 0;
        public static final double kDModuleTurnPIDValue = 0;
        public static final double kFFModuleTurnPIDValue = 0;
        public static final double kIZoneModuleTurnPIDValue = 0.5 / 360; // 1/2 degrees converted to rotations

        public static final double kPModuleSIMTurnPIDValue = 1.5;
        public static final double kIModuleSIMTurnPIDValue = 0;
        public static final double kDModuleSIMTurnPIDValue = 0;

        // If useing TalonFX Swerve these can not be set. Instead please rely on the the max and min voltage.
        public static final double kTurnMotorMinPercentOutput = -.2;
        public static final double kTurnMotorMaxPercentOutput = .2;

        public static final int kDriveMotorMaxAmpsSparkMax = 40;
        public static final int kTurnMotorMaxAmpsSparkMax = 5;
        
        public static final int kDriveMotorMaxVoltageSparkMaxTalonFX = 12;

        //TODO: This can definetly be lowered
        public static final int kTurnMotorMaxVoltageSparkMaxTalonFX = 2; 

        // Swerve Module Configuration Values
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.9064);//Units.inchesToMeters(3.733428923);// 3.83931974016;
        public static final double kDriveGearRatio = 7.125;//7.125;// 6.75/1.0;
        public static final double kTurningGearRatio = 12.8/1;

        public static final double kWheelDistancePerRotation = kWheelDiameterMeters * Math.PI;
        public static final double kDriveConversionPositionFactor = kWheelDistancePerRotation / kDriveGearRatio;
        public static final double kDriveConversionVelocityFactor = kDriveConversionPositionFactor / 60.0; /* Figure why this sixty is here. It was last seasion
            on it work so it does not need to change but why is it here. */
        public static final String kSwerveModuleOutputLoggerBase = "SwerveDrive/Modules/";
    }
}
