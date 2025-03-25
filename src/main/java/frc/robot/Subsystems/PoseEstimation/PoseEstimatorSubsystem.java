
package frc.robot.Subsystems.PoseEstimation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants.VisionConstants;
import frc.robot.Constants.MechanismConstants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class PoseEstimatorSubsystem extends SubsystemBase{
    private OdometryIO odometryIO;
    private OdometryIOInputsAutoLogged odometryInputs = new OdometryIOInputsAutoLogged();


    private PhotonCamera[] photonCameras = {new PhotonCamera("Unknown1"), new PhotonCamera("BRModuleCam")};
    private PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[photonCameras.length];

    public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem) {
        configPhotonPoseEstimators();
        SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics,
            driveSubsystem.getGyroAngleRotation2d(), driveSubsystem.getModulePositions(), new Pose2d(),
            VisionConstants.kSwerveDrivePoseEstimateTrust, VisionConstants.kVisionPoseEstimateTrust);
        VisionConstants.kAprilTagLayout.setOrigin(VisionConstants.originPosition);
        this.odometryIO = new OdometryIOUpdater(swerveDrivePoseEstimator, 
        driveSubsystem, this.photonPoseEstimators, this.photonCameras);
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
        this.photonPoseEstimators[0] = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kBackLeftCameraToCenter);
        this.photonPoseEstimators[1] = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kBackRightCameraToCenter);
    }

}
