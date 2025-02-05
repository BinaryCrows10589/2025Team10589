package frc.robot.Commands.AutoPositionCommands;

import org.littletonrobotics.junction.Logger;
import org.photonvision.jni.WpilibLoader;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Constants.GenericConstants.RobotModeConstants;
import frc.robot.Constants.GenericConstants.AutoPositionConstants.ReefPosition1Constants;
import frc.robot.Constants.GenericConstants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;


public class PlaceCoralOnReef extends Command{  
    public static enum ReefPosition {
        Position1,
        Position2,
        Position3,
        Position4,
        Position5,
        Position6,
        Position7,
        Position8,
        Position9,
        Position10,
        Position11,
        Position12,
    }
    /*
     * Goal impliment logic only once
     * be able to set fully difforent pid values for every single auto position point
     * 
     */

    private String commandName;
    private AutonPoint reefPosition;
    private double[] translationXPIDConstants;
    private double[] translationYPIDConstants;
    private double[] rotationPIDConstants;
    private Pose2d positionTolorence;
    private double maxTranslationalSpeedInMetersPerSecond;
    private double maxTranslationalAccelerationInMetersPerSecond;
    private double maxRotationalSpeedInRadsPerSecond;
    private double maxRotationalAccelerationInRadsPerSecond;
    private TrapezoidProfile.Constraints rotationPIDControllerConstraints;
    private double maxTrajectoryTime;
    private double maxScrollTime;
    private double angleOfAttack;
    
    private double[] scrollVelocityVector;
    private double[] lockRotationPIDConstants;
    
    DriveSubsystem driveSubsystem;
    ReefTreeDetectorSubsystem reefTreeDetectorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    OuttakeCommandFactory outtakeCommandFactory;
    WPILibFollowTrajectoryFromPointsCommand lineUpTrajectory;
    public PlaceCoralOnReef(ReefPosition reefPosition, DriveSubsystem driveSubsystem,
        ReefTreeDetectorSubsystem reefTreeDetectorSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        OuttakeCommandFactory outtakeCommandFactory) {
        this.driveSubsystem = driveSubsystem;
        this.reefTreeDetectorSubsystem = reefTreeDetectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.outtakeCommandFactory = outtakeCommandFactory;
        
        configureForPosition(reefPosition);
    }

    private void configureForPosition(ReefPosition reefPosition) {
        switch (reefPosition) {
            case Position1:
                this.commandName = "AutoPosition1";
                this.reefPosition = ReefPosition1Constants.kReefPosition;
                this.translationXPIDConstants = ReefPosition1Constants.kTranslationXPIDConstants;
                this.translationYPIDConstants = ReefPosition1Constants.kTranslationYPIDConstants;
                this.rotationPIDConstants = ReefPosition1Constants.kRotationPIDConstants;
                this.positionTolorence = ReefPosition1Constants.kPositionTolorence;
                this.maxTranslationalSpeedInMetersPerSecond = ReefPosition1Constants.kMaxTranslationalSpeedInMetersPerSecond;
                this.maxTranslationalAccelerationInMetersPerSecond = ReefPosition1Constants.kMaxTranslationalAccelerationInMetersPerSecond;
                this.maxRotationalSpeedInRadsPerSecond = ReefPosition1Constants.kMaxRotationalSpeedInRadsPerSecond;
                this.maxRotationalAccelerationInRadsPerSecond = ReefPosition1Constants.kMaxRotationalAccelerationInRadsPerSecond;
                this.rotationPIDControllerConstraints = ReefPosition1Constants.kRotationPIDControllerConstraints;
                this.angleOfAttack = ReefPosition1Constants.KAngleOfAttack;
                this.scrollVelocityVector = ReefPosition1Constants.kScrollVelocityVector;
                this.lockRotationPIDConstants = ReefPosition1Constants.kLockRotationPIDConstants;
                this.maxTrajectoryTime = ReefPosition1Constants.kMaxTrajectoryTime;
                this.maxScrollTime = ReefPosition1Constants.kMaxScrollTime;
                break;
                
            default:
                break;
        }
    }


    @Override
    public void initialize() {
        AutonPoint autonPoint = new AutonPoint(this.driveSubsystem.getRobotPose());
        autonPoint.setAngleOfAttack(this.angleOfAttack);
        this.reefPosition.setAngleOfAttack(this.angleOfAttack);
        AutonPoint[] pointArray = {autonPoint, this.reefPosition};
        
        this.lineUpTrajectory = new WPILibFollowTrajectoryFromPointsCommand(commandName,
            pointArray,
            this.maxTrajectoryTime,
            this.translationXPIDConstants,
            this.translationYPIDConstants,
            this.rotationPIDConstants,
            this.maxTranslationalSpeedInMetersPerSecond,
            this.maxTranslationalAccelerationInMetersPerSecond,
            this.maxRotationalSpeedInRadsPerSecond,
            this.maxRotationalAccelerationInRadsPerSecond,
            this.positionTolorence,
            driveSubsystem);

        this.lineUpTrajectory.schedule();
        
    }

    @Override
    public void execute() {
        if(!this.lineUpTrajectory.isScheduled()) {
            this.driveSubsystem.drive(this.scrollVelocityVector[0], this.scrollVelocityVector[1], this.scrollVelocityVector[2], false);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
