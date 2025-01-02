package frc.robot.Subsystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.jni.ControlConfigJNI;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.AutonConstants.PathPlannerAutonConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.PoseEstimation.PoseEstimatorSubsystem;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIO;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModule;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIO;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroIOInputsAutoLogged;

public class DriveSubsystem extends SubsystemBase{

    private final SwerveModule frontLeftSwerveModule;
    private final SwerveModule frontRightSwerveModule;
    private final SwerveModule backLeftSwerveModule;
    private final SwerveModule backRightSwerveModule;
    
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged(); 

    private SwerveModuleState[] swerveModuleStates;

    private ChassisSpeeds desiredChassisSpeeds;

    private PoseEstimatorSubsystem poseEstimatorSubsystem;

    private NetworkTablesTunablePIDConstants pathPlannerTranslationPIDValueTuner;
    private NetworkTablesTunablePIDConstants pathPlannerRotationPIDValueTuner;

    public DriveSubsystem(SwerveModuleIO frontLeftSwerveModuleIO, SwerveModuleIO frontRightSwerveModuleIO,
        SwerveModuleIO backLeftSwerveModuleIO, SwerveModuleIO backRightSwerveModuleIO, GyroIO gyroIO) {
        
        this.frontLeftSwerveModule = new SwerveModule(frontLeftSwerveModuleIO, SwerveDriveConstants.kFrontLeftModuleName);
        this.frontRightSwerveModule = new SwerveModule(frontRightSwerveModuleIO, SwerveDriveConstants.kFrontRightModuleName);
        this.backLeftSwerveModule = new SwerveModule(backLeftSwerveModuleIO, SwerveDriveConstants.kBackLeftModuleName);
        this.backRightSwerveModule = new SwerveModule(backRightSwerveModuleIO, SwerveDriveConstants.kBackRightModuleName);
        
        this.gyroIO = gyroIO;

        this.poseEstimatorSubsystem = new PoseEstimatorSubsystem(this);
        
        this.pathPlannerTranslationPIDValueTuner = new NetworkTablesTunablePIDConstants("PathPlanner/TranslationPIDValues",
            PathPlannerAutonConstants.kTranslationPIDConstants.kP,
            PathPlannerAutonConstants.kTranslationPIDConstants.kI,
            PathPlannerAutonConstants.kTranslationPIDConstants.kD, 0);

        this.pathPlannerRotationPIDValueTuner = new NetworkTablesTunablePIDConstants("PathPlanner/RotationPIDValues",
            PathPlannerAutonConstants.kRotationPIDConstants.kP,
            PathPlannerAutonConstants.kRotationPIDConstants.kI,
            PathPlannerAutonConstants.kRotationPIDConstants.kD, 0);

        configurePathPlannerAutoBuilder();
    }

    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity) {
        this.drive(desiredXVelocity, desiredYVelocity, desiredRotationalVelocity, true);
    }

    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity, boolean isFieldRelative) {
        this.desiredChassisSpeeds = isFieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity, this.poseEstimatorSubsystem.getRobotPose().getRotation()) :
            new ChassisSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity);
        
    }

    public void drive(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }
  
    public void stop() {
        this.drive(0, 0, 0);
        this.frontLeftSwerveModule.stopModuleDrive();
        this.frontRightSwerveModule.stopModuleDrive();
        this.backLeftSwerveModule.stopModuleDrive();
        this.backRightSwerveModule.stopModuleDrive();
    }

    /**
     * Sets the swerves to 45 degrees to lock the robot in place.
     */
    public void lockSwerves(){
        SwerveModuleState frontLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        SwerveModuleState frontRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        SwerveModuleState rearLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        SwerveModuleState rearRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));

        this.frontLeftSwerveModule.setDesiredModuleState(frontLeftLockupState);
        this.frontRightSwerveModule.setDesiredModuleState(frontRightLockupState);
        this.backLeftSwerveModule.setDesiredModuleState(rearLeftLockupState);
        this.backRightSwerveModule.setDesiredModuleState(rearRightLockupState);
    }

    @Override
    public void periodic() {
        this.swerveModuleStates = getModuleStates();

        if (desiredChassisSpeeds != null) {  
            SwerveModuleState[] desiredStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);   
            // If we're not trying to move, we lock the angles of the wheels
            if (desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
                && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
                SwerveModuleState[] currentStates = swerveModuleStates;
                for(int i = 0; i < currentStates.length; i++) {
                    desiredStates[i].angle = currentStates[i].angle;
                }
            }
            // Positive angles should be counter clockwise.
            Logger.recordOutput(SwerveDriveConstants.kSwerveDriveModuleStatesLoggerBase + "DesiredModuleStates", desiredStates);
            setModuleStates(desiredStates);
        }
        
        logSwerveDrive();

        // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
        desiredChassisSpeeds = null;

        updatePathPannerPIDValues();
    }

    public void updatePathPannerPIDValues() {
        double[] currentTranslationPIDValues = this.pathPlannerTranslationPIDValueTuner.getUpdatedPIDConstants();
        if(this.pathPlannerTranslationPIDValueTuner.hasAnyPIDValueChanged()) {
            PathPlannerAutonConstants.kTranslationPIDConstants = new PIDConstants(currentTranslationPIDValues[0], currentTranslationPIDValues[1], currentTranslationPIDValues[2]);
            configurePathPlannerAutoBuilder();
        }

        double[] currentRotationPIDValues = this.pathPlannerRotationPIDValueTuner.getUpdatedPIDConstants();
        if(this.pathPlannerRotationPIDValueTuner.hasAnyPIDValueChanged()) {
            PathPlannerAutonConstants.kRotationPIDConstants = new PIDConstants(currentRotationPIDValues[0], currentRotationPIDValues[1], currentRotationPIDValues[2]);
            configurePathPlannerAutoBuilder();
        }
    }

    private void logSwerveDrive() {
        ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();

        this.frontLeftSwerveModule.periodic();
        this.frontRightSwerveModule.periodic();
        this.backLeftSwerveModule.periodic();
        this.backRightSwerveModule.periodic();
        this.gyroIO.updateInputs(gyroInputs, currentChassisSpeeds.omegaRadiansPerSecond);
        Logger.processInputs("SwerveDrive/Gyro", gyroInputs);

        Logger.recordOutput(SwerveDriveConstants.kSwerveDriveChassisSpeedLoggerBase + "CurrentXVelocityMPS", currentChassisSpeeds.vxMetersPerSecond);
        Logger.recordOutput(SwerveDriveConstants.kSwerveDriveChassisSpeedLoggerBase + "CurrentYVelocityMPS", currentChassisSpeeds.vyMetersPerSecond);
        Logger.recordOutput(SwerveDriveConstants.kSwerveDriveChassisSpeedLoggerBase + "CurrentRotationVelocityRadiansPerSecond", currentChassisSpeeds.omegaRadiansPerSecond);

        if(desiredChassisSpeeds != null) {
            Logger.recordOutput(SwerveDriveConstants.kSwerveDriveDesiredChassisSpeedLoggerBase + "DesiredXVelocityMPS", this.desiredChassisSpeeds.vxMetersPerSecond);
            Logger.recordOutput(SwerveDriveConstants.kSwerveDriveDesiredChassisSpeedLoggerBase + "DesiredYVelocityMPS", this.desiredChassisSpeeds.vyMetersPerSecond);
            Logger.recordOutput(SwerveDriveConstants.kSwerveDriveDesiredChassisSpeedLoggerBase + "DesiredRotationVelocityRadiansPerSecond", this.desiredChassisSpeeds.omegaRadiansPerSecond);
        }
    
        Logger.recordOutput(SwerveDriveConstants.kSwerveDriveModuleStatesLoggerBase + "ActualModuleStates", getModuleStates());
    }

    /**
     * Gets the position of all four swerve modules
     * @return SwerveModulePosition[4]: Returns an array of the module position of the modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = {
            this.frontLeftSwerveModule.getModulePosition(),
            this.frontRightSwerveModule.getModulePosition(),
            this.backLeftSwerveModule.getModulePosition(),
            this.backRightSwerveModule.getModulePosition()
        };
        return positions;
    }

    /**
     * Gets the position of all four swerve modules
     * @return SwerveModulePosition[4]: Returns an array of the module position of the modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            this.frontLeftSwerveModule.getModuleState(),
            this.frontRightSwerveModule.getModuleState(),
            this.backLeftSwerveModule.getModuleState(),
            this.backRightSwerveModule.getModuleState()
        };
        return states;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
        this.frontLeftSwerveModule.setDesiredModuleState(desiredStates[0]);
        this.frontRightSwerveModule.setDesiredModuleState(desiredStates[1]);
        this.backLeftSwerveModule.setDesiredModuleState(desiredStates[2]);
        this.backRightSwerveModule.setDesiredModuleState(desiredStates[3]); 
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(this.swerveModuleStates);
    }

    public Rotation2d getGyroAngleRotation2d() {
        return this.gyroInputs.yawAngle;
    }

    public Pose2d getRobotPose() {
        return this.poseEstimatorSubsystem.getRobotPose();
    }

    public void setRobotPose(AutonPoint newRobotPose) {
        setRobotPose(newRobotPose.getAutonPoint());
    }

    private void setRobotPose(Pose2d newRobotPose) {
        this.poseEstimatorSubsystem.setRobotPose(newRobotPose);
    }

    public void resetRobotPose() {
        this.poseEstimatorSubsystem.resetRobotPose();
    }

    public PoseEstimatorSubsystem getPoseEstimatorSubsystem() {
        return this.poseEstimatorSubsystem;
    }

    public void setSlowModeTrue() {
        ControlConstants.slowModeActive = true;
    }

    public void setSlowModeFalse() {
        ControlConstants.slowModeActive = false;
    }

    private void configurePathPlannerAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getRobotPose, // Robot pose supplier
            this::setRobotPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    PathPlannerAutonConstants.kTranslationPIDConstants, // Translation PID constants
                    PathPlannerAutonConstants.kRotationPIDConstants, // Rotation PID constants
                    PathPlannerAutonConstants.kMaxModuleSpeedMetersPerSecond, // Max module speed, in m/s
                    SwerveDriveConstants.kRadiusFromCenterToFarthestSwerveModule, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            ()->false,
            this // Reference to this subsystem to set requirements
        );
    }
    
}
