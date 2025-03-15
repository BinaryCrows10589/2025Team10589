package frc.robot.Subsystems.AlgaeSystem.AlgaePivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.AlgaePivotConstants;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.PivotContants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class AlgaePivotIOTalonFXS implements AlgaePivotIO {
    
    private TalonFXS pivotMotor;
    private CANcoder pivotEncoder;

    private NetworkTablesTunablePIDConstants pivotMotorPIDConstantTuner;

    private double desiredPivotRotations = 0;

    public AlgaePivotIOTalonFXS() {
        this.pivotMotor = new TalonFXS(AlgaePivotConstants.kPivotMotorCANID);
        this.pivotEncoder = new CANcoder(AlgaePivotConstants.kPivotEncoderCANID);
        configurePivot();

        Timer.delay(0.1);
    }

    private void configurePivot() {
        TalonFXSConfiguration pivotConfig = new TalonFXSConfiguration();
        pivotConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.ExternalFeedback.FeedbackRemoteSensorID = ElevatorConstants.kElevatorEncoderCANID;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Voltage.PeakForwardVoltage = PivotContants.kMaxVoltage;
        pivotConfig.Voltage.PeakReverseVoltage = -PivotContants.kMaxVoltage;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaePivotConstants.kForwardSoftLimit + AlgaePivotConstants.kPivotEncoderOffset;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaePivotConstants.kReverseSoftLimit + AlgaePivotConstants.kPivotEncoderOffset;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        Slot0Configs pivotPIDConfig = new Slot0Configs();
        pivotPIDConfig.kP = AlgaePivotConstants.kPivotPPIDValue;
        pivotPIDConfig.kI = AlgaePivotConstants.kPivotIPIDValue;
        pivotPIDConfig.kD = AlgaePivotConstants.kPivotDPIDValue;

        pivotMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants(
            "/Algae/Pivot", 
            AlgaePivotConstants.kPivotPPIDValue, 
            AlgaePivotConstants.kPivotIPIDValue, 
            AlgaePivotConstants.kPivotDPIDValue,
            AlgaePivotConstants.kPivotFPIDValue);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.MagnetOffset = 0.0;
        pivotEncoder.getConfigurator().apply(encoderConfig);

        this.pivotMotor.getConfigurator().apply(pivotConfig);
        this.pivotMotor.getConfigurator().apply(pivotPIDConfig);
        
    }

    @Override
    public void setDesiredPivotRotation(double desiredRotations) {
        desiredPivotRotations = desiredRotations;
        this.pivotMotor.setControl(new PositionVoltage(desiredRotations + AlgaePivotConstants.kPivotEncoderOffset));
    }

    @Override
    public void updateInputs(AlgaePivotIOInputs pivotIOInputs) {
        pivotIOInputs.rawDesiredPivotAngleRotations = desiredPivotRotations;
        pivotIOInputs.offsetDesiredPivotAngleRotations = desiredPivotRotations + AlgaePivotConstants.kPivotEncoderOffset;
        pivotIOInputs.rawPivotAngleRotations = pivotMotor.getPosition().getValueAsDouble();
        pivotIOInputs.offsetPivotAngleRotations = pivotMotor.getPosition().getValueAsDouble() - AlgaePivotConstants.kPivotEncoderOffset;
        pivotIOInputs.pivotMotorAppliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
        pivotIOInputs.pivotMotorCurrentAmps = new double[] {pivotMotor.getSupplyCurrent().getValueAsDouble()};
        pivotIOInputs.pivotRPM = this.pivotMotor.getVelocity().getValueAsDouble();
        
        this.pivotMotorPIDConstantTuner.updatePIDValuesFromNetworkTables(this.pivotMotor);
    }

}