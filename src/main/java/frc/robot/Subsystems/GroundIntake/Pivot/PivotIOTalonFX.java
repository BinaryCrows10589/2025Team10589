package frc.robot.Subsystems.GroundIntake.Pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PivotConstants;
import frc.robot.Utils.GeneralUtils.Tolerance;

public class PivotIOTalonFX implements PivotIO{

    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private PositionVoltage desiredPivotPosition = new PositionVoltage(PivotConstants.kRotationOffset).withSlot(0);


    public PivotIOTalonFX() {
        this.pivotMotor = new TalonFX(PivotConstants.kPivotMotorCANID);
        this.pivotEncoder = new CANcoder(PivotConstants.kPivotEncoderCANID);

        configurePivotEncoder();
        configurePivotMotor();

        Timer.delay(.5);
    }

    private void configurePivotMotor() {
        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        pivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfiguration.Feedback.FeedbackRemoteSensorID = this.pivotEncoder.getDeviceID();
        pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        pivotConfiguration.Feedback.RotorToSensorRatio = PivotConstants.kPivotGearRatio;
        pivotConfiguration.Voltage.PeakForwardVoltage = PivotConstants.kMaxVoltage;
        pivotConfiguration.Voltage.PeakReverseVoltage = -PivotConstants.kMaxVoltage;

        Slot0Configs pivotPositionalPIDConfigs = new Slot0Configs();
        pivotPositionalPIDConfigs.kP = PivotConstants.kPivotPPIDValue;
        pivotPositionalPIDConfigs.kI = PivotConstants.kPivotIPIDValue;
        pivotPositionalPIDConfigs.kD = PivotConstants.kPivotDPIDValue;

        this.pivotMotor.getConfigurator().apply(pivotConfiguration);
        this.pivotMotor.getConfigurator().apply(pivotPositionalPIDConfigs);
    }

    private void configurePivotEncoder() {
        CANcoderConfiguration pivotEncoderConfigurations = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        magnetConfigs.MagnetOffset = 0.0;
        pivotEncoderConfigurations.MagnetSensor = magnetConfigs;
        this.pivotEncoder.getConfigurator().apply(pivotEncoderConfigurations);
    }

    @Override
    public void updateInputs(PivotIOInputs pivotIOInputs) {
        pivotIOInputs.pivotAngleRotations = this.pivotMotor.getPosition().getValueAsDouble();
        pivotIOInputs.pivotRPM = this.pivotMotor.getVelocity().getValueAsDouble();
        pivotIOInputs.pivotMotorAppliedVolts = this.pivotMotor.getMotorVoltage().getValueAsDouble();
        pivotIOInputs.pivotMotorCurrentAmps = new double[] {this.pivotMotor.getSupplyCurrent().getValueAsDouble()};
    }

    public void setDesiredPivotRotation(double desiredRotations) {
        this.desiredPivotPosition.Position = desiredRotations;
        this.pivotMotor.setControl(this.desiredPivotPosition);
    }

}
