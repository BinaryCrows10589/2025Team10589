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
import frc.robot.Constants.GroundIntakeConstants.PivotContants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class PivotIOTalonFX implements PivotIO{

    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private PositionVoltage desiredPivotPosition = new PositionVoltage(PivotContants.kRotationOffset).withSlot(0);

    private NetworkTablesTunablePIDConstants pivotMotorPIDConstantTuner;

    public PivotIOTalonFX() {
        this.pivotMotor = new TalonFX(PivotContants.kPivotMotorCANID);
        this.pivotEncoder = new CANcoder(PivotContants.kPivotEncoderCANID);

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
        pivotConfiguration.Feedback.RotorToSensorRatio = PivotContants.kPivotGearRatio;
        pivotConfiguration.Voltage.PeakForwardVoltage = PivotContants.kMaxVoltage;
        pivotConfiguration.Voltage.PeakReverseVoltage = -PivotContants.kMaxVoltage;

        Slot0Configs pivotPositionalPIDConfigs = new Slot0Configs();
        pivotPositionalPIDConfigs.kP = PivotContants.kPivotPPIDValue;
        pivotPositionalPIDConfigs.kI = PivotContants.kPivotIPIDValue;
        pivotPositionalPIDConfigs.kD = PivotContants.kPivotDPIDValue;

        this.pivotMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("GroundIntake/Pivot/", 
            pivotPositionalPIDConfigs.kP,
            pivotPositionalPIDConfigs.kI,
            pivotPositionalPIDConfigs.kD,
            0);

        this.pivotMotor.getConfigurator().apply(pivotConfiguration);
        this.pivotMotor.getConfigurator().apply(pivotPositionalPIDConfigs);
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentDrivePIDValues = this.pivotMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.pivotMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            Slot0Configs newDrivePIDConfigs = new Slot0Configs();
            newDrivePIDConfigs.kP = currentDrivePIDValues[0];
            newDrivePIDConfigs.kI = currentDrivePIDValues[1];
            newDrivePIDConfigs.kD = currentDrivePIDValues[2];
            newDrivePIDConfigs.kS = currentDrivePIDValues[3];
            this.pivotMotor.getConfigurator().apply(newDrivePIDConfigs);
        }
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
        pivotIOInputs.pivotRPM = this.pivotMotor.getVelocity().getValueAsDouble() * 60;
        pivotIOInputs.pivotMotorAppliedVolts = this.pivotMotor.getMotorVoltage().getValueAsDouble();
        pivotIOInputs.pivotMotorCurrentAmps = new double[] {this.pivotMotor.getSupplyCurrent().getValueAsDouble()};

        updatePIDValuesFromNetworkTables();
    }

    public void setDesiredPivotRotation(double desiredRotations) {
        this.desiredPivotPosition.Position = desiredRotations;
        this.pivotMotor.setControl(this.desiredPivotPosition);
    }

}
