package frc.robot.Subsystems.Climber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.MechanismConstants.ClimberConstants;
import frc.robot.Constants.MechanismConstants.ElevatorConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class ClimberIONeoREVThroughbore implements ClimberIO {
    
    private SparkMax climberMotor;

    private SparkClosedLoopController climberPIDController;

    private double desiredClimberPosition = 0.0;

    private NetworkTablesTunablePIDConstants climberMotorPIDConstantTuner;

    public ClimberIONeoREVThroughbore() {
        this.climberMotor = new SparkMax(ClimberConstants.kClimberMotorCANID, MotorType.kBrushless);

        configureClimberMotor();
    }

    private void configureClimberMotor() {
        SparkMaxConfig climberConfig = new SparkMaxConfig();

        climberConfig.smartCurrentLimit(ClimberConstants.kClimberSmartCurrentLimit);

        climberConfig.closedLoop.pid(
            ClimberConstants.kClimberPPIDValue,
            ClimberConstants.kClimberIPIDValue,
            ClimberConstants.kClimberDPIDValue
        );

        this.climberMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("/Climber/Motor",
            ClimberConstants.kClimberPPIDValue,
            ClimberConstants.kClimberIPIDValue,
            ClimberConstants.kClimberDPIDValue,
            0
        );
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climberPIDController = climberMotor.getClosedLoopController();
        
    }

    @Override
    public void updateInputs(ClimberIOInputs climberIOInputs) {
        climberIOInputs.climberRPM = climberMotor.getAlternateEncoder().getVelocity();
        climberIOInputs.climberPosition = climberMotor.getAlternateEncoder().getPosition();
        climberIOInputs.desiredClimberPosition = desiredClimberPosition;
        climberIOInputs.climberAppliedVolts = climberMotor.getAppliedOutput() * climberMotor.getBusVoltage();
        climberIOInputs.climberCurrentAmps = new double[] {climberMotor.getOutputCurrent()};
        updatePIDValuesFromNetworkTables();
    }

    private void updatePIDValuesFromNetworkTables() {
        double[] climberPIDValues = this.climberMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.climberMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            SparkMaxConfig newDrivePIDConfigs = new SparkMaxConfig();
            newDrivePIDConfigs.closedLoop.pid(
                climberPIDValues[0], 
                climberPIDValues[1], 
                climberPIDValues[2]);
            this.climberMotor.configure(newDrivePIDConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
    public void setDesiredPosition(double desiredPosition) {
        desiredClimberPosition = desiredPosition;
        climberPIDController.setReference(desiredPosition, ControlType.kPosition);
    }

    @Override
    public void incrementDesiredPosition(double increment) {
        desiredClimberPosition += increment;//no gear ratio? * ClimberConstants.k;
        this.climberPIDController.setReference(increment, ControlType.kPosition);
    }

}
