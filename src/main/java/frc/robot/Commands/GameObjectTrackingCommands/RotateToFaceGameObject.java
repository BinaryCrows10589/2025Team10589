package frc.robot.Commands.GameObjectTrackingCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants.GameObjectTrackingConstants.RotateToFaceGameObjectConstants;
import frc.robot.Constants.GenaricConstants.AutonConstants.PIDPositioningAutonConstants;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.GeneralUtils.Tolerance;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class RotateToFaceGameObject extends Command {
    private DriveSubsystem driveSubsystem;
    private NetworkTablesTunablePIDConstants rotationPIDTuner;
    private ProfiledPIDController rotationPIDController;
    private double goalRotation = 30000;

    public RotateToFaceGameObject(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.rotationPIDController = new ProfiledPIDController(
            RotateToFaceGameObjectConstants.kPRotationConstant,
            RotateToFaceGameObjectConstants.kIRotationConstant,
            RotateToFaceGameObjectConstants.kDRotationConstant,
            RotateToFaceGameObjectConstants.kRotationPIDControllerConstraints
        );
        this.rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPIDController.setTolerance(RotateToFaceGameObjectConstants.kRotationTolorence);
        this.rotationPIDTuner = new NetworkTablesTunablePIDConstants("RotateToFaceGameObject" + "/PID", PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant, 
            0.0);
        addRequirements(this.driveSubsystem);
    }

    public void configurePIDTuners() {
        this.rotationPIDTuner = new NetworkTablesTunablePIDConstants(
            "PIDGoToPose/Rotation",
            PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant,
            0);
    }

    /**
     * WARNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the PIDs bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentRotationPIDValues = this.rotationPIDTuner.getUpdatedPIDConstants();
        if(this.rotationPIDTuner.hasAnyPIDValueChanged()) {
            this.rotationPIDController = new ProfiledPIDController(
                currentRotationPIDValues[0],
                currentRotationPIDValues[1],
                currentRotationPIDValues[2],
                PIDPositioningAutonConstants.kRotationPIDControllerConstraints);
            this.rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
            this.rotationPIDController.setTolerance(PIDPositioningAutonConstants.kRotationToleranceRadians);
        }
    }

    @Override
    public void initialize() {
        Logger.recordOutput("RotateToFaceGameObject" + "/FacingGameObject", false);
        this.goalRotation = driveSubsystem.getRobotPose().getRotation().getRadians() + GameObjectTracker.getTargetDistanceAndHeading()[1];
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        double rotationSpeed = rotationPIDController.calculate(driveSubsystem.getRobotPose().getRotation().getRadians(), goalRotation);
        this.driveSubsystem.drive(0.0, 0.0, rotationSpeed, true);
    }

    @Override 
    public void end(boolean interrupt) {
        Logger.recordOutput("RotateToFaceGameObject" + "/FacingGameObject", true);
        this.driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
