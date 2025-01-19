package frc.robot.Subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    ElevatorIO elevatorIO;
    ElevatorIOInputsAutoLogged elevatorInputs;

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void periodic() {
        this.elevatorIO.updateInputs(this.elevatorInputs);
    }


}
