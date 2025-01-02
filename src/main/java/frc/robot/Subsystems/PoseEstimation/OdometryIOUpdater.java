package frc.robot.Subsystems.PoseEstimation;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class OdometryIOUpdater implements OdometryIO{

    private DriveSubsystem driveSubsystem;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private PhotonPoseEstimator[] photonPoseEstimators;
    private OdometryUpdaterThread odometryUpdaterThread;
    
     /**
      * Creates a OdometryIOSwerveDrivePoseEstimator object
      * @param driveSubsystem DriveSubsysem: The drive subsystem.
      */
    public OdometryIOUpdater(SwerveDrivePoseEstimator swerveDrivePoseEstimator, DriveSubsystem driveSubsystem,
        PhotonPoseEstimator... photonPoseEstimators) {
        this.driveSubsystem = driveSubsystem;
        this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;
        this.photonPoseEstimators = photonPoseEstimators;
        this.odometryUpdaterThread = new OdometryUpdaterThread(this.swerveDrivePoseEstimator, this.driveSubsystem, this.photonPoseEstimators);
       //TODO: Add this back to reenable vision
        this.odometryUpdaterThread.start(); //Comment back in to enable vision
    }

    @Override
    public void updateInputs(OdometryIOInputs inputs) {
        try {
            this.swerveDrivePoseEstimator.update(this.driveSubsystem.getGyroAngleRotation2d(),
                    this.driveSubsystem.getModulePositions());
            Logger.recordOutput("Odometry/UpdatedSuccess", true);

        } catch(Exception E) {
            Logger.recordOutput("Odometry/UpdatedSuccess", false);
        }
        
                    
        inputs.robotPosition = this.swerveDrivePoseEstimator.getEstimatedPosition();
        Logger.recordOutput("RobotPosition", inputs.robotPosition);
    }  

    public Pose2d getRobotPose() {
        return this.swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setRobotPose(AutonPoint newRobotPose) {
        setRobotPose(newRobotPose.getAutonPoint());
    }

    public void setRobotPose(Pose2d newRobotPose) {
        this.swerveDrivePoseEstimator.resetPosition(this.driveSubsystem.getGyroAngleRotation2d(),
        this.driveSubsystem.getModulePositions(), newRobotPose);
    }

    public void resetRobotPose() {
        this.swerveDrivePoseEstimator.resetPosition(this.driveSubsystem.getGyroAngleRotation2d(),
        this.driveSubsystem.getModulePositions(), new Pose2d());
    }

    public void updateAlliance() {        
        if(RobotModeConstants.hasAllianceChanged) {
            AprilTagFieldLayout fieldTags = photonPoseEstimators[0].getFieldTags();
            if(RobotModeConstants.isBlueAlliance) {
                fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                VisionConstants.originPosition = OriginPosition.kBlueAllianceWallRightSide;
            } else {
                fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                VisionConstants.originPosition = OriginPosition.kRedAllianceWallRightSide;
            }
            
            setRobotPose(flipAlliance(getRobotPose()));
            for(PhotonPoseEstimator photonPoseEstimator : this.photonPoseEstimators) {
                photonPoseEstimator.getFieldTags().setOrigin(VisionConstants.originPosition);
                Logger.recordOutput("Vision/OrginPosition", photonPoseEstimator.getFieldTags().getOrigin());
            }
            Logger.recordOutput("Vision/OrginPosition", photonPoseEstimators[0].getFieldTags().getOrigin());
        }
    }
            
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
            new Translation2d(FieldConstants.kFieldLengthMeters, FieldConstants.kFieldWidthMeters),
            new Rotation2d(Math.PI)));
    }

}