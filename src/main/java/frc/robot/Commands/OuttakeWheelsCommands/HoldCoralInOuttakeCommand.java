package frc.robot.Commands.OuttakeWheelsCommands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.jni.ControlConfigJNI;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.LEDConstants;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsIO;
import frc.robot.Subsystems.Outtake.OuttakeCoralSensors.OuttakeCoralSensorsSubsystem;
import frc.robot.Subsystems.Outtake.OuttakeWheels.OuttakeWheelsSubsystem;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.LEDUtils.LEDManager;

public class HoldCoralInOuttakeCommand extends Command {

    private final OuttakeWheelsSubsystem outtakeWheelsSubsystem;
    private final OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem;
    private final Wait hardCutOffTimer;

    public HoldCoralInOuttakeCommand(double waitTime, OuttakeWheelsSubsystem outtakeWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this.outtakeWheelsSubsystem = outtakeWheelsSubsystem;
        this.outtakeCoralSensorsSubsystem = outtakeCoralSensorsSubsystem;
        hardCutOffTimer = new Wait(waitTime);
        addRequirements(outtakeCoralSensorsSubsystem);
    }

    public HoldCoralInOuttakeCommand(OuttakeWheelsSubsystem outtakeWheelsSubsystem, OuttakeCoralSensorsSubsystem outtakeCoralSensorsSubsystem) {
        this(150, outtakeWheelsSubsystem, outtakeCoralSensorsSubsystem);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("CheckingHoldCoralIsFinishee", false);
        this.outtakeWheelsSubsystem.setWheelVoltages(OuttakeConstants.kHoldCoralVoltage);
        hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        this.outtakeWheelsSubsystem.setWheelVoltages(0);
        this.hardCutOffTimer.disableTimer();
        LEDManager.setSolidColor(ControlConstants.kCoralIntakedColor);
    }

    @Override
    public boolean isFinished() {
        Logger.recordOutput("CheckingHoldCoralIsFinishee", true);
        return this.outtakeCoralSensorsSubsystem.isCoralInEndOfOuttake(false) || hardCutOffTimer.hasTimePassed();
    }
    
    
}
