package frc.robot.Commands.AutoPositionCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.HighLevelCommandsFactory;
import frc.robot.Commands.OuttakeWheelsCommands.OuttakeCoralCommand;
import frc.robot.Constants.MechanismConstants.TransitConstants;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.TransitTunnel.TransitWheels.TransitWheelsSubsystem;
import frc.robot.Utils.CommandUtils.CustomWaitCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;
import frc.robot.Utils.CommandUtils.Wait;

public class ScrollThanOuttakeCommand extends Command {
    
    private ScrollWithReefTreeDetectorCommand scrollWithReefTreeDetectorCommand;
    private OuttakeCommandFactory outtakeCommandFactory;
    private OuttakeCoralCommand outtakeWheelsCommand;
    private CustomWaitCommand timeBeforeOuttake;
    private SequentialGroupCommand waitThenOuttake;

    public ScrollThanOuttakeCommand(ScrollWithReefTreeDetectorCommand scrollWithReefTreeDetectorCommand, double timeBeforeOutput,
     OuttakeCommandFactory outtakeCommandFactory) {
        this.scrollWithReefTreeDetectorCommand = scrollWithReefTreeDetectorCommand;
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeWheelsCommand = this.outtakeCommandFactory.createOuttakeCoralCommand();
        this.timeBeforeOuttake = new CustomWaitCommand(timeBeforeOutput);
        this.waitThenOuttake = new SequentialGroupCommand(this.timeBeforeOuttake, this.outtakeWheelsCommand);

    }
    
    @Override
    public void initialize() {
        this.scrollWithReefTreeDetectorCommand.schedule();
        Logger.recordOutput("Scroll/ScrollFinished", false);

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            this.waitThenOuttake.schedule();
        }
        this.scrollWithReefTreeDetectorCommand.cancel();
    }

    
    @Override
    public boolean isFinished() {
        Logger.recordOutput("Scroll/ScrollFinished", this.scrollWithReefTreeDetectorCommand.isFinished());
        return this.scrollWithReefTreeDetectorCommand.isFinished();
    }
}
