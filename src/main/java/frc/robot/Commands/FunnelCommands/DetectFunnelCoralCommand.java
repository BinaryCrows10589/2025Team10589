package frc.robot.Commands.FunnelCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.OuttakeWheelsCommands.HoldCoralInOuttakeCommand;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;

public class DetectFunnelCoralCommand extends Command {
    
    private final OuttakeCommandFactory outtakeCommandFactory;
    private final HoldCoralInOuttakeCommand holdCoralInOuttakeCommand;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private boolean isSameCoral = false;

    public DetectFunnelCoralCommand(
        OuttakeCommandFactory outtakeCommandFactory,
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        this.holdCoralInOuttakeCommand = this.outtakeCommandFactory.createHoldCoralInOuttakeCommand();
        addRequirements(outtakeCommandFactory.getOuttakeWheelsSubsystem());
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //this.isSameCoral = false;
        // Randomly Not indexing coral
        // Cause____
        boolean coralInStartOfOuttake = this.outtakeCoralSensorsSubsystem.isCoralInStartOfOuttake(false);
        boolean isCoralInEndOfOuttake = this.outtakeCoralSensorsSubsystem.isCoralInEndOfOuttake(false);
        if(coralInStartOfOuttake && !isCoralInEndOfOuttake && !isSameCoral) {
            this.holdCoralInOuttakeCommand.schedule();
            this.isSameCoral = true;
            
        }
        if(this.holdCoralInOuttakeCommand.isFinished() && this.isSameCoral) {
            this.isSameCoral = false;
        }
        Logger.recordOutput("Funnel/HasSeenCoral", this.isSameCoral);
    }

    @Override
    public void end(boolean interrupted) {
        this.isSameCoral = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
