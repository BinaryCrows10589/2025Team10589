package frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands;

import java.util.ArrayList;

import javax.tools.ToolProvider;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.GeneralUtils.Tolerance;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;
import frc.robot.Utils.LEDUtils.LEDManager;

public class WPILibFollowTrajectoryFromPointsCommand extends Command{
    private DriveSubsystem driveSubsystem;

    private HolonomicDriveController wpiLibDriveController;
    private Trajectory trajectory;
    private Pose2d desiredEndPoint;
    private String pathName;

    private PIDController translationXPIDController;
    private PIDController translationYPIDController;
    private ProfiledPIDController rotationPIDController;
    private NetworkTablesTunablePIDConstants translationXPIDValueTuner;
    private NetworkTablesTunablePIDConstants translationYPIDValueTuner;
    private NetworkTablesTunablePIDConstants wpiLibRotationPIDValueTuner;
    private Pose2d positionTolorance;
    private TrapezoidProfile.Constraints rotationPIDConstraints;
    private Wait hardCutOffTimer;
    private double startTime = 0;

    /**
     * @param pathName The Path Name
     * @param points The Points that make up the path
     * @param maxTime The max time that the path can run for
     * @param driveSubsystem
     */
    public WPILibFollowTrajectoryFromPointsCommand(String pathName, AutonPoint[] points, double maxTime, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.pathName = pathName;
        this.positionTolorance = WPILibAutonConstants.kPositionTolorence;
        this.hardCutOffTimer = new Wait(maxTime);

        double[] translationPIDValues = {WPILibAutonConstants.kPTranslationPIDConstant,
            WPILibAutonConstants.kITranslationPIDConstant,
            WPILibAutonConstants.kDTranslationPIDConstant};
        double[] rotationPIDValues =  {WPILibAutonConstants.kPRotationPIDConstant,
            WPILibAutonConstants.kIRotationPIDConstant,
            WPILibAutonConstants.kDRotationPIDConstant};
        this.rotationPIDConstraints = WPILibAutonConstants.kRotationPIDControllerConstraints;
        configurePIDs(translationPIDValues, translationPIDValues, rotationPIDValues);
        configureWPILibDriveController();
        setUpTrajectoryFromPoints(points, WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
            WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond);
            
        Logger.recordOutput("Auton/" + pathName +"/WPILibTrajectory/Trajectory", trajectory);

        addRequirements(this.driveSubsystem);
    }

    /**
     * @param pathName The Path Name
     * @param points The Points that make up the path
     * @param maxTime The max time that the path can run for
     * @param translationXPIDValues
     * @param translationYPIDValues
     * @param rotationPIDValues
     * @param driveSubsystem
     */
    public WPILibFollowTrajectoryFromPointsCommand(String pathName, AutonPoint[] points, double maxTime, double[] translationXPIDValues,
        double[] translationYPIDValues, double[] rotationPIDValues, double maxTranslationalSpeed, double maxTranslationalAcceleration,
        double maxRotationalSpeed, double maxRotationalAcceleration, Pose2d positionTolorance, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.pathName = pathName;
        this.positionTolorance = positionTolorance;
        this.hardCutOffTimer = new Wait(maxTime);

        this.rotationPIDConstraints = new TrapezoidProfile.Constraints(
            maxRotationalSpeed, maxRotationalAcceleration);
        configurePIDs(translationXPIDValues, translationYPIDValues, rotationPIDValues);
        configureWPILibDriveController();
        setUpTrajectoryFromPoints(points, maxRotationalSpeed,
            maxTranslationalAcceleration);
            
        Logger.recordOutput("Auton/" + pathName +"/WPILibTrajectory/Trajectory", trajectory);

        addRequirements(this.driveSubsystem);
    }

    private void configurePIDs(double[] translationXPIDValues,
        double[] translationYPIDValues, double[] rotationPIDValues) {
        
        this.translationXPIDController = new PIDController(
            translationXPIDValues[0],
            translationXPIDValues[1],
            translationXPIDValues[2]);

        this.translationYPIDController = new PIDController(
            translationYPIDValues[0],
            translationYPIDValues[1],
            translationYPIDValues[2]);

        this.rotationPIDController = new ProfiledPIDController(
            rotationPIDValues[0],
            rotationPIDValues[1],
            rotationPIDValues[2],
            this.rotationPIDConstraints);

        configurePIDTuners(translationXPIDValues, translationYPIDValues, rotationPIDValues);
    }

    public void configurePIDTuners(double[] translationXPIDValues,
        double[] translationYPIDValues, double[] rotationPIDValues) {
        this.translationXPIDValueTuner = new NetworkTablesTunablePIDConstants("Auton/" + pathName + "/TranslationXPIDValues",
            translationXPIDValues[0],
            translationXPIDValues[1],
            translationXPIDValues[2], 0);

        this.translationYPIDValueTuner = new NetworkTablesTunablePIDConstants("Auton/" + pathName + "/TranslationYPIDValues",
            translationYPIDValues[0],
            translationYPIDValues[1],
            translationYPIDValues[2], 0);

        this.wpiLibRotationPIDValueTuner = new NetworkTablesTunablePIDConstants("Auton/" + pathName + "/RotationPIDValues",
            rotationPIDValues[0],
            rotationPIDValues[1],
            rotationPIDValues[2], 0);
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the PIDs bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        boolean hasAnyPIDValueChanged = false;
        double[] currentTranslationXPIDValues = this.translationXPIDValueTuner.getUpdatedPIDConstants();
        if(this.translationXPIDValueTuner.hasAnyPIDValueChanged()) {
            this.translationXPIDController = new PIDController(
                currentTranslationXPIDValues[0],
                currentTranslationXPIDValues[1],
                currentTranslationXPIDValues[2]);
            hasAnyPIDValueChanged = true;
        }

        double[] currentTranslationYPIDValues = this.translationYPIDValueTuner.getUpdatedPIDConstants();
        if(this.translationYPIDValueTuner.hasAnyPIDValueChanged()) {
            this.translationYPIDController = new PIDController(
                currentTranslationYPIDValues[0],
                currentTranslationYPIDValues[1],
                currentTranslationYPIDValues[2]);
            hasAnyPIDValueChanged = true;
        }
       
        double[] currentRotationPIDValues = this.wpiLibRotationPIDValueTuner.getUpdatedPIDConstants();
        if(this.wpiLibRotationPIDValueTuner.hasAnyPIDValueChanged()) {
            this.rotationPIDController = new ProfiledPIDController(
                currentRotationPIDValues[0],
                currentRotationPIDValues[1],
                currentRotationPIDValues[2],
                this.rotationPIDConstraints);
            hasAnyPIDValueChanged = true;
        }
        if(hasAnyPIDValueChanged) {
            configureWPILibDriveController();
            hasAnyPIDValueChanged = false;
        }
    } 

    public void configureWPILibDriveController() {
        this.wpiLibDriveController = new HolonomicDriveController(
            this.translationXPIDController, this.translationYPIDController,
            this.rotationPIDController);
        this.wpiLibDriveController.setTolerance(WPILibAutonConstants.kPositionTolorence);
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
        this.desiredEndPoint = points[points.length-1].getAutonPoint();
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxTranslationSpeedMPS,
        maxTranslationalAccelerationInMPS);
        trajectoryConfig.setReversed(false);
        this.trajectory = TrajectoryGenerator.generateTrajectory(mirroredPoints, trajectoryConfig);
    }

    @Override 
    public void initialize() {
        Logger.recordOutput("Auton/" + pathName + "/Started", true);
        this.startTime = Timer.getFPGATimestamp();
        this.hardCutOffTimer.startTimer();
        configureWPILibDriveController();
        LEDManager.setSolidColor(new int[] {255, 0, 0});
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        Trajectory.State goalState = trajectory.sample(Timer.getFPGATimestamp()-startTime);
        ChassisSpeeds desiredChassisSpeeds = this.wpiLibDriveController.calculate(this.driveSubsystem.getRobotPose(), goalState, this.desiredEndPoint.getRotation());
        this.driveSubsystem.drive(desiredChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Auton/" + pathName + "/Started", false);
        Logger.recordOutput("Auton/" + pathName + "/PathTime", Timer.getFPGATimestamp() - startTime);
        this.driveSubsystem.stop();
        LEDManager.setSolidColor(new int[] {0, 0, 255});        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Pose2d robotPose = this.driveSubsystem.getRobotPose();
        boolean translationXInTolorence = Tolerance.inTolorance(this.desiredEndPoint.getX(),  robotPose.getX(),
            this.positionTolorance.getX());
        boolean translationYInTolorence = Tolerance.inTolorance(this.desiredEndPoint.getY(),  robotPose.getY(),
            this.positionTolorance.getY());
        boolean rotationalInTolorence = Tolerance.inTolorance(this.desiredEndPoint.getRotation().getRadians(),
            robotPose.getRotation().getRadians(),
            this.positionTolorance.getRotation().getRadians());
        return (translationXInTolorence && translationYInTolorence && rotationalInTolorence) || this.hardCutOffTimer.hasTimePassed();
    }
}