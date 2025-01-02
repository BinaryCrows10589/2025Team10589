package frc.robot.Deprecated;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.WPILibAutonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;
import frc.robot.Utils.LEDUtils.LEDManager;

public class WPILibTrajectoryCommandCreator extends Command{
    private DriveSubsystem driveSubsystem;

    private Trajectory trajectory;
    private Rotation2d desiredEndAngleRotation2d;

    private double startTime = 0;

    private PIDController translationXPIDController;
    private PIDController translationYPIDController;
    private ProfiledPIDController rotationPIDController;

    private HolonomicDriveController wpiLibDriveController;

    private NetworkTablesTunablePIDConstants wpiConstantsPIDLibTranslationPIDValueTuner;
    private NetworkTablesTunablePIDConstants wpiLibRotationPIDValueTuner;

    private TrajectoryConfig trajectoryConfig;
    private String autonName = "";

    private boolean hasSetGoal = false;

    public WPILibTrajectoryCommandCreator(String autonName, AutonPoint[] points, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.autonName = autonName;
        double[] translationPIDValues = {WPILibAutonConstants.kPTranslationPIDConstant,
            WPILibAutonConstants.kITranslationPIDConstant,
            WPILibAutonConstants.kDTranslationPIDConstant};
        double[] rotationPIDValues = {WPILibAutonConstants.kPRotationPIDConstant,
            WPILibAutonConstants.kIRotationPIDConstant,
            WPILibAutonConstants.kDRotationPIDConstant};
        intitalConfig(translationPIDValues, rotationPIDValues,
        WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
        WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond);

        setUpTrajectoryFromPoints(points, WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
        WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond);

        Logger.recordOutput("Auton/WPILibTrajectory/Trajectory", trajectory);
        addRequirements(this.driveSubsystem);
    }

    public WPILibTrajectoryCommandCreator(String autonName, Rotation2d goalEndRotation, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.autonName = autonName;
        double[] translationPIDValues = {WPILibAutonConstants.kPTranslationPIDConstant,
            WPILibAutonConstants.kITranslationPIDConstant,
            WPILibAutonConstants.kDTranslationPIDConstant};
        double[] rotationPIDValues = {WPILibAutonConstants.kPRotationPIDConstant,
            WPILibAutonConstants.kIRotationPIDConstant,
            WPILibAutonConstants.kDRotationPIDConstant};
        intitalConfig(translationPIDValues, rotationPIDValues,
        WPILibAutonConstants.kMaxRotationalSpeedInRadsPerSecond,
        WPILibAutonConstants.kMaxRotationalAccelerationInRadsPerSecond);
        
        setUpTrajectoryFromPath(autonName, goalEndRotation);

        Logger.recordOutput("Auton/WPILibTrajectory/Trajectory", trajectory);
        addRequirements(this.driveSubsystem);
    }

    public WPILibTrajectoryCommandCreator(String autonName, AutonPoint[] points,
        WPILIBTrajectoryConfig wpilibTrajectoryConfig,
        DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.autonName = autonName;
        
        intitalConfig(wpilibTrajectoryConfig.translationPIDValues,
            wpilibTrajectoryConfig.rotaitonPIDValues, 
            wpilibTrajectoryConfig.maxRotationSpeedRadsPerSecond,
            wpilibTrajectoryConfig.maxRotationAccelerationRadsPerSecond);

        setUpTrajectoryFromPoints(points, wpilibTrajectoryConfig.maxTranslationSpeedMPS,
            wpilibTrajectoryConfig.maxTranslationAccelerationMPS);

        Logger.recordOutput("Auton/WPILibTrajectory/Trajectory", trajectory);
        addRequirements(this.driveSubsystem);
    }

    public WPILibTrajectoryCommandCreator(String autonName, Rotation2d goalEndRotation,
        WPILIBTrajectoryConfig wpilibTrajectoryConfig,
        DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.autonName = autonName;
        intitalConfig(wpilibTrajectoryConfig.translationPIDValues,
            wpilibTrajectoryConfig.rotaitonPIDValues, 
            wpilibTrajectoryConfig.maxRotationSpeedRadsPerSecond,
            wpilibTrajectoryConfig.maxRotationAccelerationRadsPerSecond);
        
        setUpTrajectoryFromPath(autonName, goalEndRotation);

        Logger.recordOutput("Auton/WPILibTrajectory/Trajectory", trajectory);
        addRequirements(this.driveSubsystem);
    }

    private void intitalConfig(
        double[] translationPIDValues,
        double[] rotationPIDValues,
        double maxRotationSpeedRadsPerSecond,
        double maxRotationAccelerationRadsPerSecond) {
        configurePIDs(
            translationPIDValues,
            rotationPIDValues,
            new TrapezoidProfile.Constraints(
                maxRotationSpeedRadsPerSecond,
                maxRotationAccelerationRadsPerSecond));

        configureWPILibDriveController();
    }

    public void configurePIDs(double[] translationPIDValues, double[] rotationPIDValues, TrapezoidProfile.Constraints rotationConstraints) {
        this.translationXPIDController = new PIDController(
            translationPIDValues[0],
            translationPIDValues[1],
            translationPIDValues[2]);

        this.translationYPIDController = new PIDController(
            translationPIDValues[0],
            translationPIDValues[1],
            translationPIDValues[2]);

        this.rotationPIDController = new ProfiledPIDController(
            rotationPIDValues[0],
            rotationPIDValues[1],
            rotationPIDValues[2],
            rotationConstraints);

        configurePIDTuners(translationPIDValues, rotationPIDValues);
    }

    public void configurePIDTuners(double[] translationPIDValues, double[] rotationPIDValues) {
        this.wpiConstantsPIDLibTranslationPIDValueTuner = new NetworkTablesTunablePIDConstants("WPILib/" + this.autonName + "/TranslationPIDValues",
            translationPIDValues[0],
            translationPIDValues[1],
            translationPIDValues[2], 0);

        this.wpiLibRotationPIDValueTuner = new NetworkTablesTunablePIDConstants("WPILib/" + this.autonName + "/RotationPIDValues",
            rotationPIDValues[0],
            rotationPIDValues[1],
            rotationPIDValues[2], 0);
    }

    public void configureWPILibDriveController() {
        this.wpiLibDriveController = new HolonomicDriveController(
            this.translationXPIDController, this.translationYPIDController,
            this.rotationPIDController);
        this.wpiLibDriveController.setTolerance(WPILibAutonConstants.kPositionTolorence);
    }

    private void setUpTrajectoryFromPath(String autonName, Rotation2d goalEndRotation) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + autonName + ".wpilib.json"); 
            this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException exception) {
            this.trajectory = new Trajectory();
            Logger.recordOutput("ValidWPILIBTrajectoryLoaded", false);
        }
        this.desiredEndAngleRotation2d = goalEndRotation;
        this.trajectory = this.trajectory.transformBy(new Transform2d(
            new Translation2d(0, FieldConstants.kFieldWidthMeters * -1),
            new Rotation2d()));
        
        this.desiredEndAngleRotation2d = this.desiredEndAngleRotation2d.times(-1);
    }

    private void setUpTrajectoryFromPoints(AutonPoint[] points,
        double maxTranslationSpeedMPS,
        double maxTranslationalAccelerationInMPS) {
        ArrayList<Pose2d> mirroredPoints = new ArrayList<>();
        for(int i = 0; i < points.length; i++) {
            Pose2d mirroredPoint = points[i].getAutonPoint();
            Pose2d modifedPoint = new Pose2d(mirroredPoint.getTranslation(), new Rotation2d());
            mirroredPoints.add(modifedPoint);
        }
        this.desiredEndAngleRotation2d = points[points.length-1].getAutonPoint().getRotation();
        
        configureTrajectoryConfig(maxTranslationSpeedMPS,
            maxTranslationalAccelerationInMPS);
        this.trajectory = TrajectoryGenerator.generateTrajectory(mirroredPoints, trajectoryConfig);
    }

    
    private void configureTrajectoryConfig(double maxTranslationSpeedMPS, double maxTranslationalAccelerationInMPS) {
        trajectoryConfig = 
            new TrajectoryConfig(maxTranslationSpeedMPS,
            maxTranslationalAccelerationInMPS);
        trajectoryConfig.setReversed(false);
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the PIDs bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        boolean hasAnyPIDValueChanged = false;
        double[] currentTranslationPIDValues = this.wpiConstantsPIDLibTranslationPIDValueTuner.getUpdatedPIDConstants();
        if(this.wpiConstantsPIDLibTranslationPIDValueTuner.hasAnyPIDValueChanged()) {
            this.translationXPIDController = new PIDController(
                currentTranslationPIDValues[0],
                currentTranslationPIDValues[1],
                currentTranslationPIDValues[2]);

            this.translationYPIDController = new PIDController(
                currentTranslationPIDValues[0],
                currentTranslationPIDValues[1],
                currentTranslationPIDValues[2]);

            hasAnyPIDValueChanged = true;
        }
       
        double[] currentRotationPIDValues = this.wpiLibRotationPIDValueTuner.getUpdatedPIDConstants();
        if(this.wpiLibRotationPIDValueTuner.hasAnyPIDValueChanged()) {
            this.rotationPIDController = new ProfiledPIDController(
                currentRotationPIDValues[0],
                currentRotationPIDValues[1],
                currentRotationPIDValues[2],
                WPILibAutonConstants.kRotationPIDControllerConstraints);
            hasAnyPIDValueChanged = true;
        }
        if(hasAnyPIDValueChanged) {
            configureWPILibDriveController();
            hasAnyPIDValueChanged = false;
        }
    } 
    
    @Override 
    public void initialize() {
        Logger.recordOutput("Auton/Started", true);
        this.startTime = Timer.getFPGATimestamp();
        configureWPILibDriveController();
        LEDManager.setSolidColor(new int[] {255, 0, 0});
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        this.hasSetGoal = true;
        Trajectory.State goalState = trajectory.sample(Timer.getFPGATimestamp()-startTime);
        ChassisSpeeds desiredChassisSpeeds = this.wpiLibDriveController.calculate(this.driveSubsystem.getRobotPose(), goalState, desiredEndAngleRotation2d);
        this.driveSubsystem.drive(desiredChassisSpeeds);
        Logger.recordOutput("Auton/WPILibTrajectory/TranlsationDesiredVelXMPS", desiredChassisSpeeds.vxMetersPerSecond);
        Logger.recordOutput("Auton/WPILibTrajectory/TranlsationDesiredVelYMPS", desiredChassisSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Auton/WPILibTrajectory/RotationDesiredRadsPerSecond", desiredChassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Auton/Started", false);
        System.out.println("TrajectoryTook: " + (Timer.getFPGATimestamp()-startTime));
        this.driveSubsystem.stop();
        LEDManager.setSolidColor(new int[] {0, 0, 255});        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.wpiLibDriveController.atReference() && this.hasSetGoal;
    }
}

    

