package frc.robot.Subsystems.Outtake.OuttakeWheels;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.OuttakeConstants;

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
        outtakeWheelsIO.setWheels(0, 0);
    }
    public void outtakeCoral() {
        outtakeWheelsIO.setWheels(OuttakeConstants.kLeftWheel, -OuttakeConstants.kRightWheel);
    }
    // oh gosh, I hope it never comes to this
    public void intakeCoral() {
        outtakeWheelsIO.setWheels(-OuttakeConstants.kLeftWheel, OuttakeConstants.kRightWheel);
    }

}
