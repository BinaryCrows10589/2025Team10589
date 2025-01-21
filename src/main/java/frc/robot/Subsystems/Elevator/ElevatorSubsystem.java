package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    ElevatorIO elevatorIO;
    ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    public enum ElevatorPosition {
        L1,
        L2,
        L3,
        L4,
        FUNNEL,
        INTAKE,
        ALGAE
    }

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void periodic() {
        
        this.elevatorIO.updateInputs(this.elevatorInputs);
        Logger.processInputs("Elevator/", elevatorInputs);
    }

    public void setDesiredElevatorPosition(double desiredPosition) {
        elevatorIO.setDesiredPosition(desiredPosition);
    }
    public void setDesiredElevatorPosition(ElevatorPosition desiredPosition) {
        elevatorIO.setDesiredPosition(resolveElevatorPosition(desiredPosition));
    }

    public static double resolveElevatorPosition(ElevatorPosition desiredPosition) {
        switch (desiredPosition) {
            case L1:
                return 0;
            case L2:
                return 0;
            case L3:
                return 0;
            case L4:
                return 0;
            case FUNNEL:
                return 0;
            case INTAKE:
                return 0;
            case ALGAE:
                return 0;
            default:
                return 0;
        }
    }


}
