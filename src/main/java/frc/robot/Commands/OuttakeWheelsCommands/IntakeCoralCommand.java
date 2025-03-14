package frc.robot.Commands.OuttakeWheelsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;

public class IntakeCoralCommand extends Command{
    
    private final OuttakeWheelsSubsystem outtakeWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;

    public IntakeCoralCommand(OuttakeWheelsSubsystem outtakeWheelsSubsystem,
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
            this.outtakeWheelsSubsystem = outtakeWheelsSubsystem;
            this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
            addRequirements(this.outtakeWheelsSubsystem, this.outtakeCoralSensorsSubsystem);
    }


    @Override
    public void initialize() {  
        this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kRightWheelIntake);
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
