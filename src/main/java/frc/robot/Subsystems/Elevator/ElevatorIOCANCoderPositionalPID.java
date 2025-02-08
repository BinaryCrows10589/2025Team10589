package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ElevatorIOCANCoderPositionalPID implements ElevatorIO {
    
    private TalonFX elevatorMasterMotor;
    private TalonFX elevatorSlaveMotor;

    private CANcoder elevatorEncoder;

    private PositionVoltage desiredElevatorPosition = new PositionVoltage(ElevatorConstants.kDefaultElevatorPosition).withEnableFOC(true);

    private NetworkTablesTunablePIDConstants elevatorMotorPIDConstantTuner;


    public ElevatorIOCANCoderPositionalPID() {
        this.elevatorMasterMotor = new TalonFX(ElevatorConstants.kElevatorMasterMotorCANID);
        this.elevatorSlaveMotor = new TalonFX(ElevatorConstants.kElevatorSlaveMotorCANID);
        this.elevatorEncoder = new CANcoder(ElevatorConstants.kElevatorEncoderCANID);

        configureElevatorMotors();

        Timer.delay(.5);
    }

    private void configureElevatorMotors() {
        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        masterConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kForwardSoftLimit + ElevatorConstants.kElevatorEncoderOffset;
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        masterConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.kReverseSoftLimit + ElevatorConstants.kElevatorEncoderOffset;
        masterConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        masterConfiguration.Feedback.RotorToSensorRatio = ElevatorConstants.kElevatorGearRatio;
        masterConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        masterConfiguration.Feedback.FeedbackRemoteSensorID = ElevatorConstants.kElevatorEncoderCANID;
        masterConfiguration.Voltage.PeakForwardVoltage = ElevatorConstants.kMaxVoltage;
        masterConfiguration.Voltage.PeakReverseVoltage = -ElevatorConstants.kMaxVoltage;
        masterConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        masterConfiguration.ClosedLoopGeneral.ContinuousWrap = false;
        //masterConfiguration.Feedback.FeedbackRotorOffset = elevatorEncoder.getAbsoluteEncoder().getPosition(); // Reset the builtin encoder to the REV encoder's value

        //TODO: I don't think this requires more configuration, but we'll have to see

        Slot0Configs elevatorPositionalPIDConfigs = new Slot0Configs();
        elevatorPositionalPIDConfigs.kP = ElevatorConstants.kElevatorPPIDValue;
        elevatorPositionalPIDConfigs.kI = ElevatorConstants.kElevatorIPIDValue;
        elevatorPositionalPIDConfigs.kD = ElevatorConstants.kElevatorDPIDValue;
        elevatorPositionalPIDConfigs.kG = ElevatorConstants.kElevatorGPIDValue;
        elevatorPositionalPIDConfigs.kS = ElevatorConstants.kElevatorSPIDValue;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderConfig.MagnetSensor.MagnetOffset = 0.0;
        elevatorEncoder.getConfigurator().apply(encoderConfig);

        this.elevatorMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("Elevator/", 
            elevatorPositionalPIDConfigs.kP,
            elevatorPositionalPIDConfigs.kI,
            elevatorPositionalPIDConfigs.kD,
            0,
            elevatorPositionalPIDConfigs.kG,
            elevatorPositionalPIDConfigs.kS,
            0, 0, 0, 0, 0);

        this.elevatorMasterMotor.getConfigurator().apply(masterConfiguration);
        this.elevatorSlaveMotor.getConfigurator().apply(masterConfiguration);
        elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), ElevatorConstants.isSlaveReversed));
        this.elevatorMasterMotor.getConfigurator().apply(elevatorPositionalPIDConfigs); 
    }

    

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentDrivePIDValues = this.elevatorMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.elevatorMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            Slot0Configs newDrivePIDConfigs = new Slot0Configs();
            newDrivePIDConfigs.kP = currentDrivePIDValues[0];
            newDrivePIDConfigs.kI = currentDrivePIDValues[1];
            newDrivePIDConfigs.kD = currentDrivePIDValues[2];
            newDrivePIDConfigs.kG = currentDrivePIDValues[4];
            newDrivePIDConfigs.kS = currentDrivePIDValues[5];
            this.elevatorMasterMotor.getConfigurator().apply(newDrivePIDConfigs);
        }
    }

    @Override
    public void updateInputs(ElevatorIOInputs elevatorIOInputs) {
        elevatorIOInputs.elevatorRawPosition = elevatorMasterMotor.getPosition().getValueAsDouble();
        elevatorIOInputs.elevatorOffsetPosition = elevatorMasterMotor.getPosition().getValueAsDouble() - ElevatorConstants.kElevatorEncoderOffset;
        elevatorIOInputs.rawDesiredElevatorPosition = desiredElevatorPosition.Position;
        elevatorIOInputs.offsetDesiredElevatorPosition = getOffsetDesiredPosition().Position;
        elevatorIOInputs.elevatorMasterRPM = elevatorMasterMotor.getVelocity().getValueAsDouble();
        elevatorIOInputs.elevatorMasterAppliedVolts = elevatorMasterMotor.getMotorVoltage().getValueAsDouble();
        elevatorIOInputs.elevatorMasterCurrentAmps = new double[] {elevatorMasterMotor.getSupplyCurrent().getValueAsDouble()};
        elevatorIOInputs.elevatorMasterMotorTemputureC = this.elevatorMasterMotor.getDeviceTemp().getValueAsDouble();
        elevatorIOInputs.elevatorSlaveRPM = elevatorSlaveMotor.getVelocity().getValueAsDouble();
        elevatorIOInputs.elevatorSlaveAppliedVolts = elevatorSlaveMotor.getMotorVoltage().getValueAsDouble();
        elevatorIOInputs.elevatorSlaveCurrentAmps = new double[] {elevatorSlaveMotor.getSupplyCurrent().getValueAsDouble()};
        elevatorIOInputs.elevatorSlaveMotorTemputureC = this.elevatorSlaveMotor.getDeviceTemp().getValueAsDouble();

        updatePIDValuesFromNetworkTables();
    }

    private PositionVoltage getOffsetDesiredPosition() {
        return new PositionVoltage(desiredElevatorPosition.Position + ElevatorConstants.kElevatorEncoderOffset).withEnableFOC(true);
    }

    @Override
    public void setDesiredPosition(double desiredPosition) {
        desiredElevatorPosition.Position = desiredPosition;
        this.elevatorMasterMotor.setControl(getOffsetDesiredPosition());
        
    }

    @Override
    public void incrementDesiredPosition(double increment) {
        desiredElevatorPosition.Position += increment * ElevatorConstants.kElevatorGearRatio;
        this.elevatorMasterMotor.setControl(getOffsetDesiredPosition());
    }



}
