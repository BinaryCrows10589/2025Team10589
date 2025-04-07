package frc.robot.Commands.OuttakeWheelsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;

public class BackfeedCoralCommand extends Command{
    
    private final OuttakeWheelsSubsystem outtakeWheelsSubsystem;

    
    public BackfeedCoralCommand(OuttakeWheelsSubsystem outtakeWheelsSubsystem) {
            this.outtakeWheelsSubsystem = outtakeWheelsSubsystem;
            addRequirements(this.outtakeWheelsSubsystem);
    }


    @Override
    public void initialize() {  
        this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kBackfeedCoral);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        this.outtakeWheelsSubsystem.setWheelVoltages(0);
        
    }

    @Override
    public boolean isFinished() {
        return false;    
    }
}
