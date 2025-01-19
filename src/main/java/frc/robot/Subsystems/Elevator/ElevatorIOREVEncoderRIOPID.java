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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.GroundIntake.Pivot.PivotIO.PivotIOInputs;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ElevatorIOREVEncoderRIOPID implements ElevatorIO {
    
    private TalonFX elevatorMasterMotor;
    private TalonFX elevatorSlaveMotor;
    private SparkMax elevatorEncoder;

    private PIDController pidController;

    private double desiredElevatorPosition = ElevatorConstants.kDefaultElevatorPosition;

    private double gravityVoltageOffset = ElevatorConstants.kElevatorGPIDValue;

    private NetworkTablesTunablePIDConstants elevatorMotorPIDConstantTuner;


    public ElevatorIOREVEncoderRIOPID() {
        this.elevatorMasterMotor = new TalonFX(ElevatorConstants.kElevatorMasterMotorCANID);
        this.elevatorSlaveMotor = new TalonFX(ElevatorConstants.kElevatorSlaveMotorCANID);
        this.elevatorEncoder = new SparkMax(ElevatorConstants.kElevatorEncoderCANID, null);

        configureElevatorEncoder();
        configureElevatorMotors();
        resetElevatorMotorToAbsolute();

        Timer.delay(.5);
    }

    private void configureElevatorMotors() {
        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        masterConfiguration.Voltage.PeakForwardVoltage = ElevatorConstants.kMaxVoltage;
        masterConfiguration.Voltage.PeakReverseVoltage = -ElevatorConstants.kMaxVoltage;
        //masterConfiguration.Feedback.FeedbackRotorOffset = elevatorEncoder.getAbsoluteEncoder().getPosition(); // Reset the builtin encoder to the REV encoder's value

        //TODO: I don't think this requires more configuration, but we'll have to see
        elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), ElevatorConstants.isSlaveReversed));

        this.pidController = new PIDController(
            ElevatorConstants.kElevatorPPIDValue,
            ElevatorConstants.kElevatorIPIDValue,
            ElevatorConstants.kElevatorDPIDValue
        );

        this.elevatorMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("Elevator/", 
            pidController.getP(),
            pidController.getI(),
            pidController.getD(),
            0,
            gravityVoltageOffset,
            0
            );

        this.elevatorMasterMotor.getConfigurator().apply(masterConfiguration);
        this.elevatorSlaveMotor.getConfigurator().apply(masterConfiguration);
    }

    private void configureElevatorEncoder() {
        SparkMaxConfig encoderConfig = new SparkMaxConfig();
        elevatorEncoder.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    private void resetElevatorMotorToAbsolute() {
        this.elevatorMasterMotor.setPosition((
            this.elevatorEncoder.getAbsoluteEncoder().getPosition() * ElevatorConstants.kElevatorGearRatio
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
            pidController.setP(currentDrivePIDValues[0]);
            pidController.setI(currentDrivePIDValues[1]);
            pidController.setD(currentDrivePIDValues[2]);

            gravityVoltageOffset = currentDrivePIDValues[4];
        }
    }

    @Override
    public void updateInputs(ElevatorIOInputs elevatorIOInputs) {
        elevatorIOInputs.elevatorPosition = elevatorMasterMotor.getPosition().getValueAsDouble();
        elevatorIOInputs.desiredElevatorPosition = desiredElevatorPosition;
        elevatorIOInputs.elevatorRPM = elevatorMasterMotor.getVelocity().getValueAsDouble();
        elevatorIOInputs.elevatorAppliedVolts = elevatorMasterMotor.getMotorVoltage().getValueAsDouble();
        elevatorIOInputs.elevatorCurrentAmps = new double[] {elevatorMasterMotor.getSupplyCurrent().getValueAsDouble()};

        updatePIDValuesFromNetworkTables();
    }

    @Override
    public void setDesiredPosition(double desiredPosition) {
        desiredElevatorPosition = desiredPosition * ElevatorConstants.kElevatorGearRatio;
        this.elevatorMasterMotor.set(pidController.calculate(desiredElevatorPosition) + gravityVoltageOffset);
        
    }



}
