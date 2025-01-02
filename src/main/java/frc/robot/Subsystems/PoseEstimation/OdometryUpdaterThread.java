
package frc.robot.Subsystems.PoseEstimation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.OdometryUtils.FudgedPoint;

public class OdometryUpdaterThread extends Thread{
    private DriveSubsystem driveSubsystem;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private PhotonPoseEstimator[] photonPoseEstimators;


    public OdometryUpdaterThread(SwerveDrivePoseEstimator swerveDrivePoseEstimator, DriveSubsystem driveSubsystem, PhotonPoseEstimator[] photonPoseEstimators) {
        this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;
        this.driveSubsystem = driveSubsystem;
        this.photonPoseEstimators = photonPoseEstimators;
        
        Logger.recordOutput("Vision/Updates/VisionUpdatedSuccess", true);
    }

    @Override
    public void run() {
        while(true) {
            if(VisionConstants.updateVision) {
                try {
                    for(PhotonPoseEstimator photonPoseEstimator : this.photonPoseEstimators) {
                        photonPoseEstimator.update().ifPresent(visionReading -> {
                            boolean usedExcludedTag = false;
                            boolean highPoseAmbiguity = false;;
                            for(PhotonTrackedTarget target : visionReading.targetsUsed) {
                                for(int i = 0; i < VisionConstants.kExcludedTags.length; i++) {
                                    usedExcludedTag = target.getFiducialId() == VisionConstants.kExcludedTags[i];
                                    highPoseAmbiguity = target.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity;
                                }
                                if(usedExcludedTag || highPoseAmbiguity) {
                                        break;
                                }
                            }
                            if(visionReading != null && usedExcludedTag && !highPoseAmbiguity) {
                                Pose2d estimatedPosition = visionReading.estimatedPose.toPose2d();
                                Pose2d estimatedPositionWithGyroAngle = new Pose2d(estimatedPosition.getTranslation(),
                                    this.driveSubsystem.getGyroAngleRotation2d());
                                FudgedPoint fudgedEstimatedPosition = new FudgedPoint(estimatedPositionWithGyroAngle, VisionConstants.kFudgeFactor);
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
                Thread.sleep(100);
            } catch (InterruptedException e) {}
        }
    }
    
    private void updatedPoseEstimationWithVisionData(FudgedPoint estimatedVisionPose, double timestamp) {
        this.swerveDrivePoseEstimator.addVisionMeasurement(estimatedVisionPose.getFudgedPoint(), timestamp);
    }
    
}
