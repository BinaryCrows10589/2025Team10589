package frc.robot.Subsystems.AlgaeSystem.AlgaeWheels;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Utils.CommandUtils.Wait;


public class AlgaeWheelSubsystem extends SubsystemBase{
    AlgaeWheelIO algaeWheelIO;
    AlgaeWheelIOInputsAutoLogged algaeWheelInputs = new AlgaeWheelIOInputsAutoLogged();
    Wait algaeWheelPullInTimer;
    private boolean toggleAlgaeWheels = false;
    public AlgaeWheelSubsystem(AlgaeWheelIO algaeWheelIO) {
        this.algaeWheelIO = algaeWheelIO;
        this.algaeWheelPullInTimer = new Wait(AlgaeWheelConstants.kPullInTime);
        algaeWheelPullInTimer.startTimer();
    }

    public void periodic() {
        this.algaeWheelIO.updateInputs(this.algaeWheelInputs);
        Logger.processInputs("AlgaeSystem/Wheels", algaeWheelInputs);
        Logger.recordOutput("AlgaeSystem/Wheels/Timer", algaeWheelPullInTimer.hasTimePassed());
        if (toggleAlgaeWheels) {
            if(algaeWheelPullInTimer.hasTimePassed()) {
                algaeWheelPullInTimer.startTimer();
                if(this.algaeWheelInputs.wheelDesiredVoltage == 0) {
                    this.algaeWheelIO.setWheelVoltage(AlgaeWheelConstants.kPullInVoltage);
                } else if(this.algaeWheelInputs.wheelDesiredVoltage == AlgaeWheelConstants.kPullInVoltage) {
                    this.algaeWheelIO.setWheelVoltage(0);
                }
            }
        }
    }
    public void setAlgaeWheelPulse(boolean isPulsing) {
        toggleAlgaeWheels = isPulsing;
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
