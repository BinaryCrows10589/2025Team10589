package frc.robot.Subsystems.Elevator;

import java.util.spi.ToolProvider;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.Utils.GeneralUtils.Tolerance;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ElevatorIOCANCoderMotionMagicPIDCatch implements ElevatorIO {
    
    private TalonFX elevatorMasterMotor;
    private TalonFX elevatorSlaveMotor;

    private CANcoder elevatorEncoder;

    private MotionMagicVoltage desiredElevatorPosition = new MotionMagicVoltage(ElevatorConstants.kDefaultElevatorPosition);

    private NetworkTablesTunablePIDConstants elevatorMotorPIDConstantTuner;
    private double positionError = 0.0;
    private boolean goingDown = false;


    public ElevatorIOCANCoderMotionMagicPIDCatch() {
        this.elevatorMasterMotor = new TalonFX(ElevatorConstants.kElevatorMasterMotorCANID);
        this.elevatorSlaveMotor = new TalonFX(ElevatorConstants.kElevatorSlaveMotorCANID);
        this.elevatorEncoder = new CANcoder(ElevatorConstants.kElevatorEncoderCANID);

        configureElevatorMotors();

        Timer.delay(.2);
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


        Slot0Configs elevatorPositionalPIDConfigs = new Slot0Configs();
        elevatorPositionalPIDConfigs.kP = ElevatorConstants.kElevatorPPIDValue;
        elevatorPositionalPIDConfigs.kI = ElevatorConstants.kElevatorIPIDValue;
        elevatorPositionalPIDConfigs.kD = ElevatorConstants.kElevatorDPIDValue;
        elevatorPositionalPIDConfigs.kG = ElevatorConstants.kElevatorGPIDValue;
        elevatorPositionalPIDConfigs.kS = ElevatorConstants.kElevatorSPIDValue;
        elevatorPositionalPIDConfigs.kV = ElevatorConstants.kElevatorVPIDValue;
        elevatorPositionalPIDConfigs.kA = ElevatorConstants.kElevatorAPIDValue;
        
        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.kMotionMagicCruiseVelocity;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.kMotionMagicAcceleration;
        elevatorMotionMagicConfigs.MotionMagicJerk = ElevatorConstants.kMotionMagicJerk;

        this.elevatorMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("Elevator/", 
            elevatorPositionalPIDConfigs.kP,
            elevatorPositionalPIDConfigs.kI,
            elevatorPositionalPIDConfigs.kD,
            0,
            elevatorPositionalPIDConfigs.kG,
            elevatorPositionalPIDConfigs.kS,
            elevatorPositionalPIDConfigs.kV,
            elevatorPositionalPIDConfigs.kA,
            elevatorMotionMagicConfigs.MotionMagicCruiseVelocity,
            elevatorMotionMagicConfigs.MotionMagicAcceleration,
            elevatorMotionMagicConfigs.MotionMagicJerk
            );

        this.elevatorMasterMotor.getConfigurator().apply(masterConfiguration);
        
        this.elevatorSlaveMotor.getConfigurator().apply(masterConfiguration);
        this.elevatorMasterMotor.getConfigurator().apply(elevatorPositionalPIDConfigs); 
        elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), ElevatorConstants.isSlaveReversed));

    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentElevatorPIDValues = this.elevatorMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.elevatorMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            Slot0Configs newElevatorPIDConfigs = new Slot0Configs();
            newElevatorPIDConfigs.kP = currentElevatorPIDValues[0];
            newElevatorPIDConfigs.kI = currentElevatorPIDValues[1];
            newElevatorPIDConfigs.kD = currentElevatorPIDValues[2];
            newElevatorPIDConfigs.kG = currentElevatorPIDValues[4];
            newElevatorPIDConfigs.kS = currentElevatorPIDValues[5];
            newElevatorPIDConfigs.kV = currentElevatorPIDValues[6];
            newElevatorPIDConfigs.kA = currentElevatorPIDValues[7];

            MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
            motionMagicConfigs.MotionMagicCruiseVelocity = currentElevatorPIDValues[8];
            motionMagicConfigs.MotionMagicAcceleration = currentElevatorPIDValues[9];
            motionMagicConfigs.MotionMagicJerk = currentElevatorPIDValues[10];
            
            this.elevatorMasterMotor.getConfigurator().apply(newElevatorPIDConfigs);
            this.elevatorMasterMotor.getConfigurator().apply(motionMagicConfigs);
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
        this.positionError = getOffsetDesiredPosition().Position - elevatorIOInputs.elevatorRawPosition;
        
        elevatorIOInputs.positionError = this.positionError;

        updateElevatorControl();
        updatePIDValuesFromNetworkTables();
    }

    private PositionVoltage getOffsetDesiredPosition() {
        return new PositionVoltage(desiredElevatorPosition.Position + ElevatorConstants.kElevatorEncoderOffset);
    }

    public void disableElevatorMotors() {
        this.elevatorMasterMotor.disable();
    }

    @Override
    public void setDesiredPosition(double desiredPosition) {
        desiredElevatorPosition.Position = desiredPosition;
        ControlConstants.desiredElevatorPosition = desiredPosition;
    }

    private void updateElevatorControl() {
        double positionError = this.positionError;
        positionError = Tolerance.inTolorance(this.positionError, 0, ElevatorConstants.kCatchTolorence) ? 0 : this.positionError;
        if(Tolerance.inTolorance(this.positionError, 0, ElevatorConstants.kBasementShutoffTolerance) &&
            desiredElevatorPosition.Position == ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.BASEMENT)) {
            this.elevatorMasterMotor.stopMotor();
        } else if(positionError == 0) {
            this.elevatorMasterMotor.setControl(getOffsetDesiredPosition());
        } else if(Math.signum(positionError) == 1) {
            if(positionError < .25) {
                this.elevatorMasterMotor.setVoltage(5); //5
            } else if(positionError < .3) {
                this.elevatorMasterMotor.setVoltage(12); // 
            } else {
                this.elevatorMasterMotor.setVoltage(12); // 12
            }
        } else if(Math.signum(positionError) == -1) {
            if(desiredElevatorPosition.Position == ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.BASEMENT)) {
                this.elevatorMasterMotor.setVoltage(-4);
            } else {
                this.elevatorMasterMotor.setVoltage(-3.5);
            }
            //-3.25);
        }
    }
    
    @Override
    public void incrementDesiredPosition(double increment) {
        desiredElevatorPosition.Position += increment;
        this.elevatorMasterMotor.setControl(getOffsetDesiredPosition());
    }



}
