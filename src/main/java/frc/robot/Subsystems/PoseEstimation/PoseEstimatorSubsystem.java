
package frc.robot.Subsystems.PoseEstimation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class PoseEstimatorSubsystem extends SubsystemBase{
    private OdometryIO odometryIO;
    private OdometryIOInputsAutoLogged odometryInputs = new OdometryIOInputsAutoLogged();


    private PhotonCamera exampleCamera = new PhotonCamera("RightCamera");
    private PhotonPoseEstimator exampleEstimator;

    public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem) {
        configPhotonPoseEstimators();
        SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics,
            driveSubsystem.getGyroAngleRotation2d(), driveSubsystem.getModulePositions(), new Pose2d(),
            VisionConstants.kSwerveDrivePoseEstimateTrust, VisionConstants.kVisionPoseEstimateTrust);
        VisionConstants.kAprilTagLayout.setOrigin(VisionConstants.originPosition);
        this.odometryIO = new OdometryIOUpdater(swerveDrivePoseEstimator, 
        driveSubsystem, exampleEstimator);
    }

    public void periodic() {     
        this.odometryIO.updateInputs(this.odometryInputs);
        Logger.processInputs("Odometry/", odometryInputs);
    }

    public Pose2d getRobotPose() {
        return this.odometryInputs.robotPosition;
    }

    public void setRobotPose(AutonPoint newRobotPose) {
        setRobotPose(newRobotPose.getAutonPoint());
    }

    public void setRobotPose(Pose2d newRobotPose) {
        this.odometryIO.setRobotPose(newRobotPose);
    }

    public void resetRobotPose() {
        this.odometryIO.resetRobotPose();
    }

    public void updateAlliance() {
        this.odometryIO.updateAlliance();
    }

    private void configPhotonPoseEstimators() {
        this.exampleEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout,
            PoseStrategy.LOWEST_AMBIGUITY, exampleCamera,
            VisionConstants.kExampleCameraToRobotCenter);
    }

}
