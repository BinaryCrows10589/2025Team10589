package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.OuttakeWheelsCommands.OuttakeCoralCommand;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaeWheels.AlgaeWheelSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;
import frc.robot.Utils.CommandUtils.EndCommandAfterWait;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;


public class PrepareL1BackfeedCommand extends Command {

    private ElevatorToPositionCommand elevatorToL1Command;
    private OuttakeCoralCommand deeplyIndexCoralCommand;
    private boolean hasBegunLiftingElevator = false;

    public PrepareL1BackfeedCommand(ElevatorSubsystem elevatorSubsystem, OuttakeWheelsSubsystem outtakeWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        elevatorToL1Command = new ElevatorToPositionCommand(
            ElevatorSubsystem.resolveElevatorPosition(ElevatorSubsystem.ElevatorPosition.L1BACKFEED), 
            ElevatorConstants.kElevatorOuttakeL1Tolerance, 
            elevatorSubsystem
        );
        deeplyIndexCoralCommand = new OuttakeCoralCommand(
            OuttakeConstants.kDeeplyIndexCoralTime, 
            outtakeWheelsSubsystem, 
            outtakeCoralSensorsSubsystem);
    
        
    }

    @Override
    public void initialize() {
        deeplyIndexCoralCommand.schedule();
        elevatorToL1Command.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        elevatorToL1Command.cancel();
        deeplyIndexCoralCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}