package frc.robot.Subsystems.Outtake.OuttakeWheels;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.MechanismConstants.OuttakeConstants;

public class OuttakeWheelsSubsystem {
    
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
    public void outtakeCoral() {
        outtakeWheelsIO.setWheelVoltages(OuttakeConstants.kLeftWheel, -OuttakeConstants.kRightWheel);
    }
    // oh gosh, I hope it never comes to this
    public void intakeCoral() {
        outtakeWheelsIO.setWheelVoltages(-OuttakeConstants.kLeftWheel, OuttakeConstants.kRightWheel);
    }

}
