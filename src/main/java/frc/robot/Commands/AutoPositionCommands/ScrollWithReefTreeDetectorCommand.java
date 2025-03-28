package frc.robot.Commands.AutoPositionCommands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.SwerveDriveCommands.LockSwervesAuton;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.GenericConstants.AutoPositionConstants.AutonScrollConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;
import frc.robot.Utils.LEDUtils.LEDManager;

public class ScrollWithReefTreeDetectorCommand extends Command{
    private final double[] scrollVelocityVector;
    @SuppressWarnings("unused")
    private final double[] lockRotationPIDConstants;

    private ProfiledPIDController lockRotationController;
    private final NetworkTablesTunablePIDConstants lockRotationPIDConstantTuner;
    
    @SuppressWarnings("unused")
    private final Wait hardCutOffTimmer;

    private final DriveSubsystem driveSubsystem;
    private final LockSwervesAuton lockSwerves;
    @SuppressWarnings("unused")
    private double initialRotation;
    private BooleanSupplier isSensorInRange;
    @SuppressWarnings("unused")
    private BooleanSupplier isOtherSensorInRange;

    public ScrollWithReefTreeDetectorCommand(String pathname, double[] scrollVelocityVector, 
        double[] lockRotationPIDConstants,
        double maxScrollTime, DriveSubsystem driveSubsystem, BooleanSupplier isSensorInRange, BooleanSupplier isOtherSensorInRange) {
        this.scrollVelocityVector = scrollVelocityVector;
        this.lockRotationPIDConstants = lockRotationPIDConstants;

        this.driveSubsystem = driveSubsystem;
        this.lockRotationController = new ProfiledPIDController(lockRotationPIDConstants[0], lockRotationPIDConstants[1],
        lockRotationPIDConstants[2], AutonScrollConstants.kRotationPIDControllerConstraints);
        this.lockRotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.lockRotationPIDConstantTuner = new NetworkTablesTunablePIDConstants(pathname + "/AutonScroll", lockRotationPIDConstants[0], lockRotationPIDConstants[1],
        lockRotationPIDConstants[2], 0);
        this.hardCutOffTimmer = new Wait(maxScrollTime);
        this.lockSwerves = new LockSwervesAuton(this.driveSubsystem, AutonScrollConstants.kLockSwervesTime);
        this.isSensorInRange = isSensorInRange;
        this.isOtherSensorInRange = isOtherSensorInRange;
        addRequirements(this.driveSubsystem);
        Logger.recordOutput("Scrolling/IsScrolling", false);

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
        //this.hardCutOffTimmer.startTimer();
        this.initialRotation = this.driveSubsystem.getRobotPose().getRotation().getRadians();
        ControlConstants.isScrolling = true;
        Logger.recordOutput("Scrolling/IsScrolling", true);
        LEDManager.setSolidColor(ControlConstants.kAutoPositionColor);
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        this.driveSubsystem.drive(this.scrollVelocityVector[0], this.scrollVelocityVector[1], 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.stop();
        this.lockSwerves.schedule();
        ControlConstants.isScrolling = false;
        Logger.recordOutput("Scrolling/IsScrolling", false);

        //this.hardCutOffTimmer.disableTimer();
    }

    @Override
    public boolean isFinished() {
        Logger.recordOutput("ScrollBoolean", !this.isSensorInRange.getAsBoolean());
        return !this.isSensorInRange.getAsBoolean();// || this.hardCutOffTimmer.hasTimePassed();
    }
}
