package frc.robot.Commands.AutoPositionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.OuttakeWheelsCommands.OuttakeCoralCommand;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.TransitConstants;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsSubsystem;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;
import frc.robot.Utils.CommandUtils.Wait;

public class ScrollThanOuttakeCommand extends Command {
    
    private ScrollWithReefTreeDetectorCommand scrollWithReefTreeDetectorCommand;
    private OuttakeCommandFactory outtakeCommandFactory;
    private OuttakeCoralCommand outtakeWheelsCommand;
    private WaitCommand timeBeforeOuttake;
    private SequentialCommandGroup waitThenOuttake;

    public ScrollThanOuttakeCommand(ScrollWithReefTreeDetectorCommand scrollWithReefTreeDetectorCommand, double timeBeforeOutput,
     OuttakeCommandFactory outtakeCommandFactory) {
        this.scrollWithReefTreeDetectorCommand = scrollWithReefTreeDetectorCommand;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeWheelsCommand = this.outtakeCommandFactory.createOuttakeCoralCommand();
        this.timeBeforeOuttake = new WaitCommand(timeBeforeOutput);
        this.waitThenOuttake = new SequentialCommandGroup(
            this.timeBeforeOuttake, this.outtakeWheelsCommand);
    }
    
    @Override
    public void initialize() {
        this.scrollWithReefTreeDetectorCommand.schedule();
        ControlConstants.kIsDriverControlled = false;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            this.waitThenOuttake.schedule();
        } else {
            ControlConstants.kIsDriverControlled = true;
        }
        this.scrollWithReefTreeDetectorCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return this.scrollWithReefTreeDetectorCommand.isFinished();
    }
}
