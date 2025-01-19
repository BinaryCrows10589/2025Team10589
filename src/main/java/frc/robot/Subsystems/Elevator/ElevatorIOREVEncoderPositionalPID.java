package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIO.PivotIOInputs;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ElevatorIOREVEncoderPositionalPID implements ElevatorIO {
    
    private TalonFX elevatorMasterMotor;
    private TalonFX elevatorSlaveMotor;
    private SparkMax elevatorEncoder;

    private PositionVoltage desiredElevatorPosition = new PositionVoltage(ElevatorConstants.kDefaultElevatorPosition);

    private NetworkTablesTunablePIDConstants elevatorMotorPIDConstantTuner;


    public ElevatorIOREVEncoderPositionalPID() {
        this.elevatorMasterMotor = new TalonFX(ElevatorConstants.kElevatorMasterMotorCANID);
        this.elevatorSlaveMotor = new TalonFX(ElevatorConstants.kElevatorSlaveMotorCANID);
        this.elevatorEncoder = new SparkMax(ElevatorConstants.kElevatorEncoderCANID, null);

        configureElevatorEncoder();
        configureElevatorMotors();
    }

    private void configureElevatorMotors() {
        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        masterConfiguration.Feedback.RotorToSensorRatio = ElevatorConstants.kElevatorGearRatio;
        masterConfiguration.Voltage.PeakForwardVoltage = ElevatorConstants.kMaxVoltage;
        masterConfiguration.Voltage.PeakReverseVoltage = -ElevatorConstants.kMaxVoltage;
        //masterConfiguration.Feedback.FeedbackRotorOffset = elevatorEncoder.getAbsoluteEncoder().getPosition(); // Reset the builtin encoder to the REV encoder's value

        //TODO: I don't think this requires more configuration, but we'll have to see
        elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), ElevatorConstants.isSlaveReversed));

        Slot0Configs elevatorPositionalPIDConfigs = new Slot0Configs();
        elevatorPositionalPIDConfigs.kP = ElevatorConstants.kElevatorPPIDValue;
        elevatorPositionalPIDConfigs.kI = ElevatorConstants.kElevatorIPIDValue;
        elevatorPositionalPIDConfigs.kD = ElevatorConstants.kElevatorDPIDValue;
        elevatorPositionalPIDConfigs.kG = ElevatorConstants.kElevatorGPIDValue;
        elevatorPositionalPIDConfigs.kS = ElevatorConstants.kElevatorSPIDValue;

        this.elevatorMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("GroundIntake/Pivot/", 
            elevatorPositionalPIDConfigs.kP,
            elevatorPositionalPIDConfigs.kI,
            elevatorPositionalPIDConfigs.kD,
            0,
            elevatorPositionalPIDConfigs.kG,
            elevatorPositionalPIDConfigs.kS
            );

        this.elevatorMasterMotor.getConfigurator().apply(masterConfiguration);
        this.elevatorMasterMotor.getConfigurator().apply(elevatorPositionalPIDConfigs); 
    }

    private void configureElevatorEncoder() {
        SparkMaxConfig encoderConfig = new SparkMaxConfig();
        encoderConfig.absoluteEncoder.positionConversionFactor(ElevatorConstants.kElevatorGearRatio); //TODO: Is this right?
        elevatorEncoder.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    public void resetElevatorMotorToAbsolute() {
        this.elevatorMasterMotor.setPosition((
            this.elevatorEncoder.getAbsoluteEncoder().getPosition()
        ));
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
        elevatorIOInputs.elevatorPosition = elevatorMasterMotor.getPosition().getValueAsDouble();
        elevatorIOInputs.desiredElevatorPosition = desiredElevatorPosition.Position;
        elevatorIOInputs.elevatorRPM = elevatorMasterMotor.getVelocity().getValueAsDouble();
        elevatorIOInputs.elevatorAppliedVolts = elevatorMasterMotor.getMotorVoltage().getValueAsDouble();
        elevatorIOInputs.elevatorCurrentAmps = new double[] {elevatorMasterMotor.getSupplyCurrent().getValueAsDouble()};

        updatePIDValuesFromNetworkTables();
    }

    @Override
    public void setDesiredPosition(double desiredPosition) {
        desiredElevatorPosition.Position = desiredPosition;
        this.elevatorMasterMotor.setControl(desiredElevatorPosition);
        
    }



}
