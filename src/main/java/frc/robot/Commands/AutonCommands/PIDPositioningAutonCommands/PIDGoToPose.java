package frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.AutonConstants.PIDPositioningAutonConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;
import frc.robot.Utils.LEDUtils.LEDManager;

public class PIDGoToPose extends Command{
    private ProfiledPIDController xTranslationPIDController;
    private ProfiledPIDController yTranslationPIDController;
    private ProfiledPIDController rotationPIDController;

    private AutonPoint endPoint;

    private DriveSubsystem driveSubsystem;

    private NetworkTablesTunablePIDConstants translationPIDTuner;
    private NetworkTablesTunablePIDConstants rotationPIDTuner;

    private double startTime = 0;

    public PIDGoToPose(AutonPoint endPoint, DriveSubsystem driveSubsystem) {
        this.endPoint = endPoint;
        this.driveSubsystem = driveSubsystem;
        Logger.recordOutput("Auton/GoToPosePIDGoalEndPose", endPoint.getAutonPoint());
        
        configurePIDs();
        configurePIDTuners();
        addRequirements(this.driveSubsystem);
    }

    public void configurePIDs() {
        this.xTranslationPIDController = new ProfiledPIDController(
            PIDPositioningAutonConstants.kPTranslationPIDConstant,
            PIDPositioningAutonConstants.kITranslationPIDConstant,
            PIDPositioningAutonConstants.kDTranslationPIDConstant,
            PIDPositioningAutonConstants.kTranslationPIDControllerConstraints);

        this.yTranslationPIDController = new ProfiledPIDController(
            PIDPositioningAutonConstants.kPTranslationPIDConstant,
            PIDPositioningAutonConstants.kITranslationPIDConstant,
            PIDPositioningAutonConstants.kDTranslationPIDConstant,
            PIDPositioningAutonConstants.kTranslationPIDControllerConstraints);

        this.rotationPIDController = new ProfiledPIDController(
            PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant,
            PIDPositioningAutonConstants.kTranslationPIDControllerConstraints);

        this.rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.xTranslationPIDController.setTolerance(PIDPositioningAutonConstants.kTranslationToleranceMeters);
        this.yTranslationPIDController.setTolerance(PIDPositioningAutonConstants.kTranslationToleranceMeters);
        this.rotationPIDController.setTolerance(PIDPositioningAutonConstants.kRotationToleranceRadians);
    }

    public void configurePIDTuners() {
        this.translationPIDTuner = new NetworkTablesTunablePIDConstants(
            "PIDGoToPose/Translation",
            PIDPositioningAutonConstants.kPTranslationPIDConstant,
            PIDPositioningAutonConstants.kITranslationPIDConstant,
            PIDPositioningAutonConstants.kDTranslationPIDConstant,
            0);
        

        this.rotationPIDTuner = new NetworkTablesTunablePIDConstants(
            "PIDGoToPose/Rotation",
            PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant,
            0);
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the PIDs bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentTranslationPIDValues = this.translationPIDTuner.getUpdatedPIDConstants();
        if(this.translationPIDTuner.hasAnyPIDValueChanged()) {
            this.xTranslationPIDController = new ProfiledPIDController(
                currentTranslationPIDValues[0],
                currentTranslationPIDValues[1],
                currentTranslationPIDValues[2],
                PIDPositioningAutonConstants.kTranslationPIDControllerConstraints);
            this.yTranslationPIDController = new ProfiledPIDController(
                currentTranslationPIDValues[0],
                currentTranslationPIDValues[1],
                currentTranslationPIDValues[2],
                PIDPositioningAutonConstants.kTranslationPIDControllerConstraints);

            this.xTranslationPIDController.setTolerance(PIDPositioningAutonConstants.kTranslationToleranceMeters);
            this.yTranslationPIDController.setTolerance(PIDPositioningAutonConstants.kTranslationToleranceMeters);
        }

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
        ControlConstants.kIsDriverControlled = false;
    
        this.xTranslationPIDController.reset(this.driveSubsystem.getRobotPose().getX());
        this.yTranslationPIDController.reset(this.driveSubsystem.getRobotPose().getY());
        this.rotationPIDController.reset(this.driveSubsystem.getRobotPose().getRotation().getDegrees());

        this.startTime = Timer.getFPGATimestamp();

        LEDManager.setSolidColor(new int[] {255, 0, 0});
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();

        Pose2d currentRobotPose = this.driveSubsystem.getRobotPose();
        double xVelocity = this.xTranslationPIDController.calculate(currentRobotPose.getX(),
            endPoint.getAutonPoint().getX());
        double yVelocity = this.yTranslationPIDController.calculate(currentRobotPose.getY(),
            endPoint.getAutonPoint().getY());
        double rotationVelocity = this.rotationPIDController.calculate(currentRobotPose.getRotation().getRadians(),
            endPoint.getAutonPoint().getRotation().getRadians());
        this.driveSubsystem.drive(xVelocity, yVelocity, rotationVelocity);

        Logger.recordOutput("Auton/PIDGoToPose/TranlsationDesiredVelXMPS", xVelocity);
        Logger.recordOutput("Auton/PIDGoToPose/TranlsationDesiredVelYMPS", yVelocity);
        Logger.recordOutput("Auton/PIDGoToPose/RotationDesiredRadsPerSecond", rotationVelocity);
        
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.stop();
        LEDManager.setSolidColor(new int[] {0, 0, 255});
        ControlConstants.kIsDriverControlled = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.xTranslationPIDController.atGoal() &&
            this.yTranslationPIDController.atGoal() &&
            this.rotationPIDController.atGoal();    
    }
}
