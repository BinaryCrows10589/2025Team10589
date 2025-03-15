package frc.robot.Subsystems.Outtake.OuttakeWheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeWheelsSubsystem extends SubsystemBase{
    
    private OuttakeWheelsIO outtakeWheelsIO;
    private OuttakeWheelsIOInputsAutoLogged outtakeWheelsInputs = new OuttakeWheelsIOInputsAutoLogged();

    public OuttakeWheelsSubsystem(OuttakeWheelsIO outtakeWheelsIO) {
        this.outtakeWheelsIO = outtakeWheelsIO;
    }

    public void periodic() {
        this.outtakeWheelsIO.updateInputs(this.outtakeWheelsInputs);
        Logger.processInputs("Outtake/Wheels", outtakeWheelsInputs);
    }
    public void stopOuttake() {
        outtakeWheelsIO.setWheelVoltages( 0);
    }

    public void setWheelVoltages(double rightWheel) {
        outtakeWheelsIO.setWheelVoltages(rightWheel);
    }
    
    public void setWheelPositionsReletiveToCurrentPose(double rightWheel) {
        outtakeWheelsIO.setWheelPositionsRelative(rightWheel);
    }

}
