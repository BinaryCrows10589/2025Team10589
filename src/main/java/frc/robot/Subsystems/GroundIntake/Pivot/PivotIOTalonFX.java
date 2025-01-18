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
import frc.robot.Constants.GroundIntakeConstants;

public class PivotIOTalonFX implements PivotIO{

    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private PositionVoltage desiredPivotPosition = new PositionVoltage(GroundIntakeConstants.kRotationOffset).withSlot(0);


    public PivotIOTalonFX() {
        this.pivotMotor = new TalonFX(GroundIntakeConstants.kPivotMotorCANID);
        this.pivotEncoder = new CANcoder(GroundIntakeConstants.kPivotEncoderCANID);

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
        pivotConfiguration.Feedback.RotorToSensorRatio = GroundIntakeConstants.kPivotGearRatio;
        pivotConfiguration.Voltage.PeakForwardVoltage = GroundIntakeConstants.kMaxVoltage;
        pivotConfiguration.Voltage.PeakReverseVoltage = -GroundIntakeConstants.kMaxVoltage;

        Slot0Configs pivotPositionalPIDConfigs = new Slot0Configs();
        pivotPositionalPIDConfigs.kP = GroundIntakeConstants.kPivotPPIDValue;
        pivotPositionalPIDConfigs.kI = GroundIntakeConstants.kPivotIPIDValue;
        pivotPositionalPIDConfigs.kD = GroundIntakeConstants.kPivotDPIDValue;

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
        pivotIOInputs.desiredPivotAngleRotations = desiredPivotPosition.Position;
        pivotIOInputs.pivotRPM = this.pivotMotor.getVelocity().getValueAsDouble();
        pivotIOInputs.pivotMotorAppliedVolts = this.pivotMotor.getMotorVoltage().getValueAsDouble();
        pivotIOInputs.pivotMotorCurrentAmps = new double[] {this.pivotMotor.getSupplyCurrent().getValueAsDouble()};
    }

    public void setDesiredPivotRotation(double desiredRotations) {
        this.desiredPivotPosition.Position = desiredRotations;
        this.pivotMotor.setControl(this.desiredPivotPosition);
    }

}
