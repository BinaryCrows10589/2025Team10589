package frc.robot.Commands.OuttakeWheelsCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.LEDUtils.LEDManager;

public class OuttakeCoralCommand extends Command{
    
    private final OuttakeWheelsSubsystem outtakeWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private final Wait hardCutOffTimer;

    public OuttakeCoralCommand(double maxTime, OuttakeWheelsSubsystem outtakeWheelsSubsystem,
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
            this.outtakeWheelsSubsystem = outtakeWheelsSubsystem;
            this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
            this.hardCutOffTimer = new Wait(maxTime);
            addRequirements(this.outtakeWheelsSubsystem, this.outtakeCoralSensorsSubsystem);
    }

    public OuttakeCoralCommand(OuttakeWheelsSubsystem outtakeWheelsSubsystem,
        OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
            this(150, outtakeWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }


    @Override
    public void initialize() {  
        this.hardCutOffTimer.startTimer();
        if(ControlConstants.desiredElevatorPosition == ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L4)) {
            this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kRightWheelOuttakeL4);
        } else if(ControlConstants.desiredElevatorPosition == ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L3)){
            this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kRightWheelOuttakeL3);
        } else if(ControlConstants.desiredElevatorPosition == ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L2)){
            this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kRightWheelOuttakeL2);
        } else {
            this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kRightWheelOuttake);
        }
        Logger.recordOutput("Outtake/IsRunningOuttakeCommand", true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        this.outtakeWheelsSubsystem.setWheelVoltages(0);
        this.hardCutOffTimer.disableTimer();
        ControlConstants.kIsDriverControlled = true;
        LEDManager.setSolidColor(ControlConstants.kCoralOuttakedColor);
        
    }

    @Override
    public boolean isFinished() {
        Logger.recordOutput("Checking is finished", true);
        return !this.outtakeCoralSensorsSubsystem.isCoralInStartOfOuttake(true) && !this.outtakeCoralSensorsSubsystem.isCoralInEndOfOuttake(true);
    }
}
