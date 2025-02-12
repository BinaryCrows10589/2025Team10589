package frc.robot.Commands.SwerveDriveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.DrivetrainConstants;
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
    private double translationMultiplier = 1;
    private double rotationMultiplier = 1;
    private int elevatorCheckFrameCount = 0;
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
            double translationX = ControlConstants.slowModeActive ? 
            this.translationXSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond * ControlConstants.kTranslationXSlowModeMultipler :
            this.translationXSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
            double translationY = ControlConstants.slowModeActive ? 
            this.translationYSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond * ControlConstants.kTranslationYSlowModeMultipler : 
            this.translationYSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond; 
            double rotation = ControlConstants.slowModeActive ? 
            this.rotationSupplier.getAsDouble() * SwerveDriveConstants.kMaxRotationAnglePerSecond * ControlConstants.kRotationSlowModeMultipler : 
            this.rotationSupplier.getAsDouble() * SwerveDriveConstants.kMaxRotationAnglePerSecond;


            Logger.recordOutput("SwerveDrive/Inputs/InputedXSpeedMPS", translationX);
            Logger.recordOutput("SwerveDrive/Inputs/InputedYSpeedMPS", translationY);
            Logger.recordOutput("SwerveDrive/Inputs/InputedRotationSpeed", rotation);

            double position = m_elevatorSubsystem.getCurrentElevatorPosition();
            
            if (elevatorCheckFrameCount++ >= SwerveDriveConstants.kframesPerCheck) {
                elevatorCheckFrameCount = 0;

                translationMultiplier = 1;
                rotationMultiplier = 1;

                for (int level = SwerveDriveConstants.kElevatorThresholds.length-1; level >= 0; level--) {
                    if (position > SwerveDriveConstants.kElevatorThresholds[level]) {
                        translationMultiplier = SwerveDriveConstants.kElevatorThresholdVelocityMultipliers[level];
                        rotationMultiplier = SwerveDriveConstants.kElevatorThresholdRotationMultipliers[level];
                        break;
                    }
                }
            }

            m_driveSubsystem.drive(translationX * translationMultiplier, translationY * translationMultiplier, rotation * rotationMultiplier);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stop();
    }
}