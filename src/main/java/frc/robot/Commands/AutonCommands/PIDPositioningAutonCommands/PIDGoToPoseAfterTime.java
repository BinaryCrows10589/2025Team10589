package frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class PIDGoToPoseAfterTime extends Command{
   

    public static Command createPIDGoToPoseAfterTime(AutonPoint endPoint, DriveSubsystem driveSubsystem, double waitTime) {
        CustomWaitCommand waitTimer = new CustomWaitCommand(waitTime);
        PIDGoToPose goToPose = new PIDGoToPose(endPoint, driveSubsystem);
        Command[] goToPoseAfterTimeSequence = {waitTimer, goToPose};
        return new SequentialGroupCommand(goToPoseAfterTimeSequence);
    }

    
}
