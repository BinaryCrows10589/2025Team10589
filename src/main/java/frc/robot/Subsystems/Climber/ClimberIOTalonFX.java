package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.MechanismConstants.ClimberConstants;
import frc.robot.Constants.MechanismConstants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.Subsystems.Climber.ClimberIO.ClimberIOInputs;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ClimberIOTalonFX implements ClimberIO{
    
    private TalonFX climberMotor;
    private double desiredVoltage;
    private NetworkTablesTunablePIDConstants climberTunablePIDConstants;

    public ClimberIOTalonFX() {
        configureClimberMotor();
    }

    private void configureClimberMotor() {
        this.climberMotor = new TalonFX(ClimberConstants.kClimberMotorCANID);
        TalonFXConfiguration climberConfiguration = new TalonFXConfiguration();
        climberConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // May need to set more fields becuase optimize useage 
        this.climberMotor.getVelocity().setUpdateFrequency(20);
        this.climberMotor.getAcceleration().setUpdateFrequency(20);
        this.climberMotor.getPosition().setUpdateFrequency(20);
        this.climberMotor.getTorqueCurrent().setUpdateFrequency(50);
    
        climberConfiguration.Voltage.PeakForwardVoltage = ClimberConstants.kClimberMaxVoltage;
        climberConfiguration.Voltage.PeakReverseVoltage = -ClimberConstants.kClimberMaxVoltage;
        climberConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kMaxForwardRotations;
        climberConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kMaxReverseRotations;
        climberConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        Slot0Configs holdPositionPIDConstants = new Slot0Configs();
        holdPositionPIDConstants.kP = ClimberConstants.kClimberPPIDValue;
        holdPositionPIDConstants.kI = ClimberConstants.kClimberIPIDValue;
        holdPositionPIDConstants.kD = ClimberConstants.kClimberDPIDValue;
        

        this.climberTunablePIDConstants = new NetworkTablesTunablePIDConstants(
            "Climber/ClimberPIDValues", holdPositionPIDConstants.kP, holdPositionPIDConstants.kI, holdPositionPIDConstants.kD, 0);


        //this.climberMotor.optimizeBusUtilization();
        this.climberMotor.getConfigurator().apply(climberConfiguration);
        this.climberMotor.getConfigurator().apply(holdPositionPIDConstants);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberRPM = this.climberMotor.getVelocity().getValueAsDouble();
        inputs.climberMotorAppliedVolts = this.climberMotor.getMotorVoltage().getValueAsDouble();
        inputs.climberDesiredVoltage = this.desiredVoltage;

        this.climberTunablePIDConstants.updatePIDValuesFromNetworkTables(climberMotor);
    }

    @Override
    public void setDesiredClimberMotorVoltage(double desiredVoltage) {
        this.desiredVoltage = desiredVoltage;
        this.climberMotor.setVoltage(desiredVoltage);
        if(this.desiredVoltage == 0) {
            this.climberMotor.setControl(new PositionVoltage(this.climberMotor.getPosition().getValueAsDouble()));
        }
    }

}
