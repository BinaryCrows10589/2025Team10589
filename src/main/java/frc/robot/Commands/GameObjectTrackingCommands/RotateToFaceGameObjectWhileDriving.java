package frc.robot.Commands.GameObjectTrackingCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.PIDPositioningAutonConstants;
import frc.robot.Constants.GameObjectTrackingConstants.RotateToFaceGameObjectConstants;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.GeneralUtils.Tolerance;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class RotateToFaceGameObjectWhileDriving extends Command {
    private DriveSubsystem driveSubsystem;
    private NetworkTablesTunablePIDConstants rotationPIDTuner;
    private ProfiledPIDController rotationPIDController;
    private double goalRotation = 30000;
    private Wait hardCutOffTimer;
    private double velocityMPS;
    private double anlgeOffsetDegrees;
    private Pose2d startPosition;

    public RotateToFaceGameObjectWhileDriving(double velocityMPS, double maxTime, double anlgeOffsetDegrees, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.rotationPIDController = new ProfiledPIDController(
            RotateToFaceGameObjectConstants.kPRotationConstant,
            RotateToFaceGameObjectConstants.kIRotationConstant,
            RotateToFaceGameObjectConstants.kDRotationConstant,
            RotateToFaceGameObjectConstants.kRotationPIDControllerConstraints
        );
        this.rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPIDController.setTolerance(RotateToFaceGameObjectConstants.kRotationTolorence);
        this.rotationPIDTuner = new NetworkTablesTunablePIDConstants("RotateToFaceGameObjectWhileDriving" + "/PID", PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant, 
            0.0);

        this.hardCutOffTimer = new Wait(maxTime);
        this.velocityMPS = velocityMPS;
        this.anlgeOffsetDegrees = anlgeOffsetDegrees;
        
        addRequirements(this.driveSubsystem);
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
        //this.goalRotation = driveSubsystem.getRobotPose().getRotation().getRadians() + GameObjectTracker.getTargetDistanceAndHeading()[1] +
        //    Units.degreesToRadians(anlgeOffsetDegrees);
        this.startPosition = this.driveSubsystem.getRobotPose();
        this.hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        double dX = GameObjectTracker.getTargetDistanceAndHeading()[2] - (this.startPosition.getX() - this.driveSubsystem.getRobotPose().getX());
        double dY = GameObjectTracker.getTargetDistanceAndHeading()[3] - (this.startPosition.getY() - this.driveSubsystem.getRobotPose().getY());
        this.goalRotation = Math.atan(dX/dY);
        Logger.recordOutput("RotateToFaceGameObjectWhileDriving/GoalRotation", this.goalRotation);
        double rotationSpeed = rotationPIDController.calculate(driveSubsystem.getRobotPose().getRotation().getRadians(), goalRotation);
        this.driveSubsystem.drive(this.velocityMPS, 0.0, rotationSpeed, false);
    }

    @Override 
    public void end(boolean interrupt) {
        this.driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return this.hardCutOffTimer.hasTimePassed();
    }

}
