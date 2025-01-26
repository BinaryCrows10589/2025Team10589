package frc.robot.Subsystems.AlgaeSystem.AlgaePivot;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Subsystems.AlgaeSystem.AlgaePivot.AlgaePivotIO.AlgaePivotIOInputs;
import frc.robot.Utils.GeneralUtils.Tolerance;

public class AlgaePivotSubsystem {
    AlgaePivotIO algaePivotIO;
    AlgaePivotIOInputsAutoLogged algaePivotInputs = new AlgaePivotIOInputsAutoLogged();

    public AlgaePivotSubsystem(AlgaePivotIO algaePivotIO) {
        this.algaePivotIO = algaePivotIO;
    }

    public void periodic() {
        this.algaePivotIO.updateInputs(this.algaePivotInputs);
        Logger.processInputs("AlgaeSystem/AlgaePivot/", algaePivotInputs);
    }

    public void setDesiredPivotRotation(double desiredRotations) {
        this.algaePivotIO.setDesiredPivotRotation(desiredRotations);
    }

    public boolean isPivotInTolerance() {
        return Tolerance.inTolorance(
            this.algaePivotInputs.offsetPivotAngleRotations, 
            this.algaePivotInputs.offsetDesiredPivotAngleRotations,
            AlgaePivotConstants.kPivotAngleToleranceRotations);
    }
}
