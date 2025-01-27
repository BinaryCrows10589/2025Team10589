package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ElevatorIOCANCoderMotionMagic implements ElevatorIO {
    
    private TalonFX elevatorMasterMotor;
    private TalonFX elevatorSlaveMotor;

    private CANcoder elevatorEncoder;

    private MotionMagicVoltage desiredElevatorPosition = new MotionMagicVoltage(ElevatorConstants.kDefaultElevatorPosition);

    private NetworkTablesTunablePIDConstants elevatorMotorPIDConstantTuner;


    public ElevatorIOCANCoderMotionMagic() {
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
        masterConfiguration.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kMotionMagicCruiseVelocity;
        masterConfiguration.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kMotionMagicAcceleration;
        masterConfiguration.MotionMagic.MotionMagicJerk = ElevatorConstants.kMotionMagicJerk;

        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.MagnetOffset = 0.0;
        elevatorEncoder.getConfigurator().apply(encoderConfig);
        
        
        
        //masterConfiguration.Feedback.FeedbackRotorOffset = elevatorEncoder.getAbsoluteEncoder().getPosition(); // Reset the builtin encoder to the REV encoder's value

        //TODO: I don't think this requires more configuration, but we'll have to see
        elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), ElevatorConstants.isSlaveReversed));

        Slot0Configs elevatorPositionalPIDConfigs = new Slot0Configs();
        elevatorPositionalPIDConfigs.kP = ElevatorConstants.kElevatorPPIDValue;
        elevatorPositionalPIDConfigs.kI = ElevatorConstants.kElevatorIPIDValue;
        elevatorPositionalPIDConfigs.kD = ElevatorConstants.kElevatorDPIDValue;
        elevatorPositionalPIDConfigs.kG = ElevatorConstants.kElevatorGPIDValue;
        elevatorPositionalPIDConfigs.kS = ElevatorConstants.kElevatorSPIDValue;

        this.elevatorMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("Elevator/", 
            elevatorPositionalPIDConfigs.kP,
            elevatorPositionalPIDConfigs.kI,
            elevatorPositionalPIDConfigs.kD,
            0,
            elevatorPositionalPIDConfigs.kG,
            elevatorPositionalPIDConfigs.kS);

        

        this.elevatorMasterMotor.getConfigurator().apply(masterConfiguration);
        
        this.elevatorSlaveMotor.getConfigurator().apply(masterConfiguration);
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
        elevatorIOInputs.elevatorRPM = elevatorMasterMotor.getVelocity().getValueAsDouble();
        elevatorIOInputs.elevatorAppliedVolts = elevatorMasterMotor.getMotorVoltage().getValueAsDouble();
        elevatorIOInputs.elevatorCurrentAmps = new double[] {elevatorMasterMotor.getSupplyCurrent().getValueAsDouble()};

        updatePIDValuesFromNetworkTables();
    }

    private PositionVoltage getOffsetDesiredPosition() {
        return new PositionVoltage(desiredElevatorPosition.Position + ElevatorConstants.kElevatorEncoderOffset);
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
