package frc.robot.Subsystems.TransitTunnel.TransitWheels;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitWheelsSubsystem extends SubsystemBase{
    TransitWheelsIO transitWheelsIO;
    TransitWheelsIOInputsAutoLogged transitWheelsInputs = new TransitWheelsIOInputsAutoLogged();

    public TransitWheelsSubsystem(TransitWheelsIO pivotIO) {
        this.transitWheelsIO = pivotIO;
    }

    public void periodic() {
        this.transitWheelsIO.updateInputs(this.transitWheelsInputs);
        Logger.processInputs("Transit/TransitWheels/", this.transitWheelsInputs);
    }

    public void setDesiredTransitWheelsVoltage(double desiredVoltage) {
        this.transitWheelsIO.setDesiredTransitWheelsMotorVoltage(desiredVoltage);
    }
}
