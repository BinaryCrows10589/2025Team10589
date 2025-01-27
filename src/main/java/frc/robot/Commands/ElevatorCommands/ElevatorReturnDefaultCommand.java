package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorMode;

public class ElevatorReturnDefaultCommand extends Command {

    /*
     * This command will be the default command for the elevator subsystem
     * It needs to return the elevator to its default position when:
     * - We are in teleop
     * - We have not used manual controls (or they have been overridden by normal controls)
     */

    private final ElevatorSubsystem elevatorSubsystem;

    public ElevatorReturnDefaultCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        
    }

    private boolean shouldReturnToDefaultPosition() {
        return 
        !DriverStation.isAutonomous() &&
        elevatorSubsystem.getControlMode() == ElevatorMode.AUTOMATIC_POSITIONING;

    }

    @Override
    public void execute() {
        if (shouldReturnToDefaultPosition()) {
            elevatorSubsystem.setDesiredElevatorPosition(ElevatorConstants.kDefaultElevatorPosition);
        }
    }
    
}
