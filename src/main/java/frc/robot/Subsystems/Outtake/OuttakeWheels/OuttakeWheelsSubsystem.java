package frc.robot.Subsystems.Outtake.OuttakeWheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;

public class OuttakeWheelsSubsystem extends SubsystemBase{
    
    OuttakeWheelsIO outtakeWheelsIO;
    OuttakeWheelsIOInputsAutoLogged outtakeWheelsInputs = new OuttakeWheelsIOInputsAutoLogged();

    public OuttakeWheelsSubsystem(OuttakeWheelsIO outtakeWheelsIO) {
        this.outtakeWheelsIO = outtakeWheelsIO;
    }

    public void periodic() {
        this.outtakeWheelsIO.updateInputs(this.outtakeWheelsInputs);
        Logger.processInputs("Outtake/Wheels", outtakeWheelsInputs);
    }
    public void stopOuttake() {
        outtakeWheelsIO.setWheelVoltages(0, 0);
    }

    public void setWheelVoltages(double leftWheel, double rightWheel) {
        outtakeWheelsIO.setWheelVoltages(leftWheel, rightWheel);
    }
    
    public void setWheelPositionsReletiveToCurrentPose(double leftWheel, double rightWheel) {
        // Boyne do this
    }

}
