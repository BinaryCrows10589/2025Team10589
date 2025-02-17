package frc.robot.Commands.AutoPositionCommands;

import java.security.CodeSource;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.AutoPositionConstants.AutonScrollConstants;
import frc.robot.Subsystems.ReefTreeDetector.ReefTreeCoralDetector.ReefTreeDetectorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ScrollWithReefTreeDetectorCommand extends Command{
    private final double[] scrollVelocityVector;
    private final double[] lockRotationPIDConstants;

    private ProfiledPIDController lockRotationController;
    private final NetworkTablesTunablePIDConstants lockRotationPIDConstantTuner;
    
    private final Wait hardCutOffTimmer;

    private final DriveSubsystem driveSubsystem;
    private double initialRotation;
    private BooleanSupplier isSensorInRange;

    public ScrollWithReefTreeDetectorCommand(String pathname, double[] scrollVelocityVector, 
        double[] lockRotationPIDConstants,
        double maxScrollTime, DriveSubsystem driveSubsystem, BooleanSupplier isSensorInRange) {
        this.scrollVelocityVector = scrollVelocityVector;
        this.lockRotationPIDConstants = lockRotationPIDConstants;

        this.driveSubsystem = driveSubsystem;
        this.lockRotationController = new ProfiledPIDController(lockRotationPIDConstants[0], lockRotationPIDConstants[1],
        lockRotationPIDConstants[2], AutonScrollConstants.kRotationPIDControllerConstraints);
        this.lockRotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.lockRotationPIDConstantTuner = new NetworkTablesTunablePIDConstants(pathname + "/AutonScroll", lockRotationPIDConstants[0], lockRotationPIDConstants[1],
        lockRotationPIDConstants[2], 0);
        this.hardCutOffTimmer = new Wait(maxScrollTime);
        this.isSensorInRange = isSensorInRange;
        addRequirements(this.driveSubsystem);

    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentLockRotationPIDConstants = lockRotationPIDConstantTuner.getUpdatedPIDConstants();
        if(lockRotationPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.lockRotationController = new ProfiledPIDController(currentLockRotationPIDConstants[0],
            currentLockRotationPIDConstants[1],
            currentLockRotationPIDConstants[2],
            AutonScrollConstants.kRotationPIDControllerConstraints);
            this.lockRotationController.enableContinuousInput(-Math.PI, Math.PI);
        }
    }

    @Override
    public void initialize() {
        this.hardCutOffTimmer.startTimer();
        this.initialRotation = this.driveSubsystem.getRobotPose().getRotation().getRadians();

    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        this.driveSubsystem.drive(this.scrollVelocityVector[0], this.scrollVelocityVector[1],
            0, false);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.stop();
        this.driveSubsystem.lockSwerves();
        this.hardCutOffTimmer.disableTimer();
    }

    @Override
    public boolean isFinished() {
        return !this.isSensorInRange.getAsBoolean() || this.hardCutOffTimmer.hasTimePassed();
    }
}

