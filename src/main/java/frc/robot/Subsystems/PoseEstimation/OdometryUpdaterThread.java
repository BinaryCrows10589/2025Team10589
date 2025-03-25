
package frc.robot.Subsystems.PoseEstimation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CameraConstants.VisionConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.GeneralUtils.Tolerance;
import frc.robot.Utils.OdometryUtils.FudgedPoint;

public class OdometryUpdaterThread extends Thread{
    @SuppressWarnings("unused")
    private DriveSubsystem driveSubsystem;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private PhotonPoseEstimator[] photonPoseEstimators;
    private PhotonCamera[] photonCameras;

    public OdometryUpdaterThread(SwerveDrivePoseEstimator swerveDrivePoseEstimator, DriveSubsystem driveSubsystem,
        PhotonPoseEstimator[] photonPoseEstimators, PhotonCamera[] photonCameras) {
        this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;
        this.driveSubsystem = driveSubsystem;
        this.photonPoseEstimators = photonPoseEstimators;
        this.photonCameras = photonCameras;
        
        Logger.recordOutput("Vision/Updates/VisionUpdatedSuccess", true);
    }

    @Override
    public void run() {
        while(true) {
            if(VisionConstants.updateVision) {
                try {
                    for(int i = 0; i < this.photonPoseEstimators.length; i++) {
                        photonPoseEstimators[i].update(this.photonCameras[i].getLatestResult()).ifPresent(visionReading -> {
                            boolean usedExcludedTag = false;
                            boolean highPoseAmbiguity = false;
                            for(PhotonTrackedTarget target : visionReading.targetsUsed) {
                                for(int j = 0; j < VisionConstants.kExcludedTags.length; j++) {
                                    usedExcludedTag = target.getFiducialId() == VisionConstants.kExcludedTags[j];
                                    highPoseAmbiguity = target.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity;
                                }
                                if(usedExcludedTag || highPoseAmbiguity) {
                                    break;
                                }
                            }
                            if(visionReading != null && !usedExcludedTag && !highPoseAmbiguity) {
                                Pose2d estimatedPosition = visionReading.estimatedPose.toPose2d();
                               // Pose2d estimatedPositionWithGyroAngle = new Pose2d(estimatedPosition.getTranslation(),
                                //    this.driveSubsystem.getGyroAngleRotation2d());
                                FudgedPoint fudgedEstimatedPosition = new FudgedPoint(estimatedPosition, VisionConstants.kFudgeFactor);
                                updatedPoseEstimationWithVisionData(fudgedEstimatedPosition, visionReading.timestampSeconds);
                            }
                        });
                    }
                    Logger.recordOutput("Vision/Updates/VisionUpdatedSuccess", true);   
                } catch (Exception e) {
                    Logger.recordOutput("Vision/Updates/VisionUpdatedSuccess", false);
                }
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {}
        }
    }
    
    private void updatedPoseEstimationWithVisionData(FudgedPoint estimatedVisionPose, double timestamp) {
        if(!DriverStation.isEnabled() ||
            Tolerance.inTolerancePose2dTotalPose(driveSubsystem.getRobotPose(),
            estimatedVisionPose.getFudgedPoint(),
            VisionConstants.kAcceptableVisionDelta)) {
                this.swerveDrivePoseEstimator.addVisionMeasurement(estimatedVisionPose.getFudgedPoint(), timestamp);
            }
        }
    }
    