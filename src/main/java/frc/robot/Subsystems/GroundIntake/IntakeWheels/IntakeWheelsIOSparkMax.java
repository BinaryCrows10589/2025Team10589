package frc.robot.Subsystems.GroundIntake.IntakeWheels;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.MechanismConstants.GroundIntakeConstants.IntakeWheelsConstants;

public class IntakeWheelsIOSparkMax implements IntakeWheelsIO {
    SparkMax intakeWheelsMotor;

    public IntakeWheelsIOSparkMax() {
        configureIntakeWheelsMotor();
    }

    public void configureIntakeWheelsMotor() {
        this.intakeWheelsMotor = new SparkMax(IntakeWheelsConstants.kIntakeWheelsMotorCANID, MotorType.kBrushless);
        SparkMaxConfig intakeWheelsConfig = new SparkMaxConfig();
        intakeWheelsConfig.inverted(false);
        intakeWheelsConfig.idleMode(IdleMode.kBrake);
        intakeWheelsConfig.smartCurrentLimit(40);
    }

    @Override
    public void updateInputs(IntakeWheelsIOInputs inputs) {
        inputs.intakeWheelsRPM = this.intakeWheelsMotor.getEncoder().getVelocity();
        inputs.intakeWheelsMotorAppliedVolts = this.intakeWheelsMotor.getAppliedOutput() * this.intakeWheelsMotor.getBusVoltage();
        inputs.intakeWheelsMotorCurrentAmps = new double[] {this.intakeWheelsMotor.getOutputCurrent()};
    }

    @Override
    public void setDesiredIntakeWheelsMotorVoltage(double desiredVoltage) {
        this.intakeWheelsMotor.setVoltage(desiredVoltage);
    }

}
