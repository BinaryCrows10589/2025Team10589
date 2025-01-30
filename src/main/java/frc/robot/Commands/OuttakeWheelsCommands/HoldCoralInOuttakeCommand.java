package frc.robot.Commands.OuttakeWheelsCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class HoldCoralInOuttakeCommand extends Command {

    private final OuttakeWheelsSubsystem outtakeWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private final Wait hardCutOffTimer;

    public HoldCoralInOuttakeCommand(double waitTime, OuttakeWheelsSubsystem outtakeWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this.outtakeWheelsSubsystem = outtakeWheelsSubsystem;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        hardCutOffTimer = new Wait(waitTime);
        addRequirements(outtakeWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }

    public HoldCoralInOuttakeCommand(OuttakeWheelsSubsystem outtakeWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this(150, outtakeWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }

    @Override
    public void initialize() {
        this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kHoldCoralVoltage, OuttakeConstants.kHoldCoralVoltage);
        hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        this.outtakeWheelsSubsystem.setWheelVoltages(0, 0);
        this.hardCutOffTimer.disableTimer();
        
    }

    @Override
    public boolean isFinished() {
        Logger.recordOutput("CheckingHoldCoralIsFinishee", true);
        return this.outtakeCoralSensorsSubsystem.isCoralInEndOfOuttake(false) || hardCutOffTimer.hasTimePassed();
    }
    
}
