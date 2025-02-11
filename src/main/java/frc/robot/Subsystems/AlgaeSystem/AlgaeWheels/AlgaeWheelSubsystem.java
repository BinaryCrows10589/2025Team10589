package frc.robot.Subsystems.AlgaeSystem.AlgaeWheels;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Utils.CommandUtils.Wait;


public class AlgaeWheelSubsystem {
    AlgaeWheelIO algaeWheelIO;
    AlgaeWheelIOInputsAutoLogged algaeWheelInputs = new AlgaeWheelIOInputsAutoLogged();
    Wait algaeWheelPullInTimer;
    public AlgaeWheelSubsystem(AlgaeWheelIO algaeWheelIO) {
        this.algaeWheelIO = algaeWheelIO;
        this.algaeWheelPullInTimer = new Wait(AlgaeWheelConstants.kPullInTime);
        algaeWheelPullInTimer.startTimer();
    }

    public void periodic() {
        this.algaeWheelIO.updateInputs(this.algaeWheelInputs);
        Logger.processInputs("AlgaeSystem/Wheels", algaeWheelInputs);
        if(algaeWheelPullInTimer.hasTimePassed()) {
            algaeWheelPullInTimer.startTimer();
            if(this.algaeWheelInputs.wheelDesiredVoltage == 0) {
                this.algaeWheelIO.setWheelVoltage(AlgaeWheelConstants.kPullInVoltage);
            } else if(this.algaeWheelInputs.wheelDesiredVoltage == 0) {
                this.algaeWheelIO.setWheelVoltage(AlgaeWheelConstants.kPullInVoltage);
            }
        }
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
