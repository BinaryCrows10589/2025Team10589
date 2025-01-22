package frc.robot.Subsystems.GroundIntake.Pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Utils.GeneralUtils.Tolerance;

public class PivotSubsystem extends SubsystemBase{
    PivotIO pivotIO;
    PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

    public PivotSubsystem(PivotIO pivotIO) {
        this.pivotIO = pivotIO;
    }

    public void periodic() {
        this.pivotIO.updateInputs(this.pivotInputs);
        Logger.processInputs("GroundIntake/Pivot/", this.pivotInputs);
    }

    public void setDesiredPivotRotation(double desiredRotations) {
        this.pivotIO.setDesiredPivotRotation(desiredRotations);
    }

    public boolean isPivotInTolorence() {
        return Tolerance.inTolorance(this.pivotInputs.pivotAngleRotations, this.pivotInputs.desiredPivotAngleRotations,
        PivotContants.kPivotAngleToloranceRotations);
    }
}
