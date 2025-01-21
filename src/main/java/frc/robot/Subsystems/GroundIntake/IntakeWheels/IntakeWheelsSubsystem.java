package frc.robot.Subsystems.GroundIntake.IntakeWheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheelsSubsystem extends SubsystemBase{
    IntakeWheelsIO intakeWheelsIO;
    IntakeWheelsIOInputsAutoLogged intakeWheelsInputs = new IntakeWheelsIOInputsAutoLogged();

    public IntakeWheelsSubsystem(IntakeWheelsIO pivotIO) {
        this.intakeWheelsIO = pivotIO;
    }

    public void periodic() {
        this.intakeWheelsIO.updateInputs(this.intakeWheelsInputs);
        Logger.processInputs("GroundIntake/IntakeWheels/", this.intakeWheelsInputs);
    }

    public void setDesiredIntakeWheelsVoltage(double desiredVoltage) {
        this.intakeWheelsIO.setDesiredIntakeWheelsMotorVoltage(desiredVoltage);
    }
}
