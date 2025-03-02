package frc.robot.Subsystems.AlgaeSystem.AlgaePivot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class AlgaePivotIOSparkMax implements AlgaePivotIO {
    
    private SparkMax pivotMotor;

    private NetworkTablesTunablePIDConstants pivotMotorPIDConstantTuner;

    private SparkClosedLoopController pivotPIDController;

    private double desiredPivotRotations = 0;

    public AlgaePivotIOSparkMax() {
        this.pivotMotor = new SparkMax(AlgaePivotConstants.kPivotMotorCANID, MotorType.kBrushless);

        configurePivotMotor();

        Timer.delay(0.5);
    }

    private void configurePivotMotor() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.absoluteEncoder.setSparkMaxDataPortConfig();
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        pivotConfig.absoluteEncoder.inverted(false);
        pivotConfig.inverted(false);
        pivotConfig.smartCurrentLimit(AlgaePivotConstants.kSmartCurrentLimit);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.softLimit.forwardSoftLimit(AlgaePivotConstants.kForwardSoftLimit + PivotContants.kRotationOffset);
        pivotConfig.softLimit.reverseSoftLimit(AlgaePivotConstants.kReverseSoftLimit + PivotContants.kRotationOffset);
    
        pivotConfig.closedLoop.pid(
            AlgaePivotConstants.kPivotPPIDValue, 
            AlgaePivotConstants.kPivotIPIDValue,
            AlgaePivotConstants.kPivotDPIDValue);

        pivotMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants(
            "/Algae/Pivot", 
            AlgaePivotConstants.kPivotPPIDValue, 
            AlgaePivotConstants.kPivotIPIDValue, 
            AlgaePivotConstants.kPivotDPIDValue,
            AlgaePivotConstants.kPivotFPIDValue);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotPIDController = pivotMotor.getClosedLoopController();
    }

    @Override
    public void setDesiredPivotRotation(double desiredRotations) {
        desiredPivotRotations = desiredRotations;

        pivotPIDController.setReference(desiredRotations + AlgaePivotConstants.kPivotEncoderOffset, ControlType.kPosition);
    }

    @Override
    public void updateInputs(AlgaePivotIOInputs pivotIOInputs) {
        pivotIOInputs.rawDesiredPivotAngleRotations = desiredPivotRotations;
        pivotIOInputs.offsetDesiredPivotAngleRotations = desiredPivotRotations + AlgaePivotConstants.kPivotEncoderOffset;
        pivotIOInputs.rawPivotAngleRotations = pivotMotor.getAbsoluteEncoder().getPosition();
        pivotIOInputs.offsetPivotAngleRotations = pivotMotor.getAbsoluteEncoder().getPosition() - AlgaePivotConstants.kPivotEncoderOffset;
        pivotIOInputs.pivotMotorAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        pivotIOInputs.pivotMotorCurrentAmps = new double[] {pivotMotor.getOutputCurrent()};
        pivotIOInputs.pivotRPM = pivotMotor.getAbsoluteEncoder().getVelocity();
        
        updatePIDValuesFromNetworkTables();
    }

    private void updatePIDValuesFromNetworkTables() {
        double[] pivotPIDValues = pivotMotorPIDConstantTuner.getUpdatedPIDConstants();
        if (pivotMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            SparkMaxConfig newPIDConfig = new SparkMaxConfig();
            newPIDConfig.closedLoop.pidf(
                pivotPIDValues[0],
                pivotPIDValues[1],
                
                pivotPIDValues[2],
                pivotPIDValues[3]
            );
            this.pivotMotor.configure(newPIDConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}
