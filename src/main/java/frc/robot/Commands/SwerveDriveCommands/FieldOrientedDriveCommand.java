package frc.robot.Commands.SwerveDriveCommands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

/**
 * Command for teleop driving where translation is field oriented and rotation
 * velocity is controlled by the driver.
 * 
 * Translation is specified on the field-relative coordinate system. The Y-axis
 * runs parallel to the alliance wall, left
 * is positive. The X-axis runs down field toward the opposing alliance wall,
 * away from the alliance wall is positive.
 */
public class FieldOrientedDriveCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private double translationMax = SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    private boolean normalizeTranslationMaximum = false;
    private double rotationMax = SwerveDriveConstants.kMaxRotationAnglePerSecond;
    /**
     * Constructor
     * 
     * @param m_driveSubsystem     drivetrain
     * @param robotAngleSupplier   supplier for the current angle of the robot
     * @param translationXSupplier supplier for translation X component, in meters
     *                             per second
     * @param translationYSupplier supplier for translation Y component, in meters
     *                             per second
     * @param rotationSupplier     supplier for rotation component, in radians per
     *                             second
     */
    public FieldOrientedDriveCommand(
        DriveSubsystem m_driveSubsystem, ElevatorSubsystem m_elevatorSubsystem,
        DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, 
        DoubleSupplier rotationSupplier) {

        this.m_driveSubsystem = m_driveSubsystem;
        this.m_elevatorSubsystem = m_elevatorSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(ControlConstants.kIsDriverControlled) {
            double translationX;
            double translationY;
            double rotation;
            if (m_driveSubsystem.inTrackingMode && m_driveSubsystem.hasTarget) {
                double[] gameObjectPositionRelative = m_driveSubsystem.gameObjectPosition;
                double totalDistanceToObject = gameObjectPositionRelative[0];
                double headingToAdd = gameObjectPositionRelative[1];
                double distanceToObjectX = gameObjectPositionRelative[2];
                double distanceToObjectY = gameObjectPositionRelative[3];
                translationX = distanceToObjectX;
                translationY = distanceToObjectY;
                rotation = headingToAdd;
                normalizeTranslationMaximum = true;

                translationMax = 1;
                rotationMax = 2;
            } else {
                translationX = ControlConstants.slowModeActive ? 
                this.translationXSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond * ControlConstants.kTranslationXSlowModeMultipler :
                this.translationXSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
                translationY = ControlConstants.slowModeActive ? 
                this.translationYSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond * ControlConstants.kTranslationYSlowModeMultipler : 
                this.translationYSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond; 
                rotation = ControlConstants.slowModeActive ? 
                this.rotationSupplier.getAsDouble() * SwerveDriveConstants.kMaxRotationAnglePerSecond * ControlConstants.kRotationSlowModeMultipler : 
                this.rotationSupplier.getAsDouble() * SwerveDriveConstants.kMaxRotationAnglePerSecond;
                normalizeTranslationMaximum = false;

                
                translationMax = SwerveDriveConstants.kMaxSpeedMetersPerSecond;
                rotationMax = SwerveDriveConstants.kMaxRotationAnglePerSecond;
            }


            Logger.recordOutput("SwerveDrive/Inputs/InputedXSpeedMPS", translationX);
            Logger.recordOutput("SwerveDrive/Inputs/InputedYSpeedMPS", translationY);
            Logger.recordOutput("SwerveDrive/Inputs/InputedRotationSpeed", rotation);

            double position = m_elevatorSubsystem.getCurrentElevatorPosition();
            

            

            // Determine if we need to be applying a speed limiter to our swerve drive due to the elevator's position
            for (int level = SwerveDriveConstants.kElevatorThresholds.length-1; level >= 0; level--) {
                if (position > SwerveDriveConstants.kElevatorThresholds[level]) {
                    translationMax = SwerveDriveConstants.kElevatorThresholdVelocityCaps[level];
                    normalizeTranslationMaximum = true;
                    rotationMax = SwerveDriveConstants.kElevatorThresholdRotationCaps[level];
                    break;
                }
            }

            double translationXSpeed;
            double translationYSpeed;
            
            if (normalizeTranslationMaximum) {
                double normalizedSpeed = Math.sqrt(Math.pow(translationX, 2) + Math.pow(translationY, 2));
                if(normalizedSpeed != 0) {
                    double translationFactor = MathUtil.clamp(normalizedSpeed, -translationMax, translationMax) / normalizedSpeed;
                    translationXSpeed = translationX * translationFactor;
                    translationYSpeed = translationY * translationFactor;
                } else {
                    translationXSpeed = MathUtil.clamp(translationX, -translationMax, translationMax);
                    translationYSpeed = MathUtil.clamp(translationY, -translationMax, translationMax);
                }
               
            } else { 
                translationXSpeed = MathUtil.clamp(translationX, -translationMax, translationMax);
                translationYSpeed = MathUtil.clamp(translationY, -translationMax, translationMax);
           } 
            double rotationSpeed = MathUtil.clamp(rotation, -rotationMax, rotationMax);

            
            Logger.recordOutput("SwerveDrive/Inputs/ClampedXSpeedMPS", translationXSpeed);
            Logger.recordOutput("SwerveDrive/Inputs/ClampedYSpeedMPS", translationYSpeed);
            Logger.recordOutput("SwerveDrive/Inputs/ClampedRotSpeedMPS", rotationSpeed);


            if(ControlConstants.axisLockMode) {
                m_driveSubsystem.drive(-translationXSpeed, -translationYSpeed, rotationSpeed, false);
            } else {
                m_driveSubsystem.drive(translationXSpeed, translationYSpeed, rotationSpeed,true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stop();
    }
}