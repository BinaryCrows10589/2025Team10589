package frc.robot.Deprecated;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;
import frc.robot.Utils.CommandUtils.Wait;

public class PIDGoToObjectCommand extends Command {

    private DriveSubsystem driveSubsystem;

    private PIDGoToPose goToPoseCommand;
    private double[] targetPoseAndHeading;
    private boolean foundTarget = false;
    private Wait hardCutOffTimer;

     public PIDGoToObjectCommand(DriveSubsystem driveSubsystem, double maxTime) {
        this.driveSubsystem = driveSubsystem;
        this.hardCutOffTimer = new Wait(maxTime);
        addRequirements(this.driveSubsystem);
    }

    public PIDGoToObjectCommand(DriveSubsystem driveSubsystem) {
        this(driveSubsystem, 120);
    }

   

    @Override
    public void initialize() {
        // Find target position
        this.targetPoseAndHeading = GameObjectTracker.getTargetDistanceAndHeading(); // Returns hypotenuse, heading, x distance and y distance
        // Check that the pose isn't just zeroes
        this.foundTarget = !(
            this.targetPoseAndHeading[0] == 0 &&
            this.targetPoseAndHeading[1] == 0 &&
            this.targetPoseAndHeading[2] == 0 &&
            this.targetPoseAndHeading[3] == 0 
        );
        // Create position command
        if (foundTarget) {
            Pose2d robotPose = this.driveSubsystem.getRobotPose();
            this.goToPoseCommand = new PIDGoToPose(
                new AutonPoint(new Pose2d(
                    new Translation2d(
                        targetPoseAndHeading[2] + robotPose.getX(),
                        targetPoseAndHeading[3] + robotPose.getY()
                    ),
                    Rotation2d.fromRadians(this.targetPoseAndHeading[1]).plus(robotPose.getRotation())
                ), 
                new FudgeFactor(
                    0, 
                    0, 
                    0)
                ), // Perhaps fudge factor needs to be tuned 
                this.driveSubsystem);
                this.goToPoseCommand.initialize();
        }
        this.hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {
        if (foundTarget) {
            this.goToPoseCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.goToPoseCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.hardCutOffTimer.hasTimePassed();
    }

}
