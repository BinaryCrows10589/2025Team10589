package frc.robot.Commands.AutoPositionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.OuttakeWheelsCommands.OuttakeCoralCommand;
import frc.robot.Constants.MechanismConstants.TransitConstants;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class ScrollThanOuttakeCommand extends Command {
    
    private ScrollWithReefTreeDetectorCommand scrollWithReefTreeDetectorCommand;
    private OuttakeCommandFactory outtakeCommandFactory;
    private OuttakeCoralCommand outtakeWheelsCommand;

    public ScrollThanOuttakeCommand(ScrollWithReefTreeDetectorCommand scrollWithReefTreeDetectorCommand,
     OuttakeCommandFactory outtakeCommandFactory) {
        this.scrollWithReefTreeDetectorCommand = scrollWithReefTreeDetectorCommand;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeWheelsCommand = this.outtakeCommandFactory.createOuttakeCoralCommand();
    }
    
    @Override
    public void initialize() {
        this.scrollWithReefTreeDetectorCommand.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            //this.outtakeWheelsCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return this.scrollWithReefTreeDetectorCommand.isFinished();
    }
}
