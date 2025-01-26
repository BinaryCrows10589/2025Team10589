package frc.robot.Subsystems.AlgaeSystem.AlgaeWheels;

import org.littletonrobotics.junction.Logger;


public class AlgaeWheelSubsystem {
    AlgaeWheelIO algaeWheelIO;
    AlgaeWheelIOInputsAutoLogged algaeWheelInputs = new AlgaeWheelIOInputsAutoLogged();

    public AlgaeWheelSubsystem(AlgaeWheelIO algaeWheelIO) {
        this.algaeWheelIO = algaeWheelIO;
    }

    public void periodic() {
        this.algaeWheelIO.updateInputs(this.algaeWheelInputs);
        Logger.processInputs("AlgaeSystem/Wheels", algaeWheelInputs);
    }
    public void stopWheel() {
        algaeWheelIO.setWheelVoltage(0);
    }

    public void setWheelVoltage(double voltage) {
        algaeWheelIO.setWheelVoltage(voltage);
    }
    
    public void setWheelPositionsReletiveToCurrentPose(double position) {
        algaeWheelIO.setWheelPositionRelative(position);
    }
}
