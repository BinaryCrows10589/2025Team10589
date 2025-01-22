package frc.robot.Subsystems.TransitTunnel.TransitWheels;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.MechanismConstants.TransitConstants.TransitWheelsConstants;

public class TransitWheelsIOSparkMax implements TransitWheelsIO {
    SparkMax transitWheelsMotor;

    public TransitWheelsIOSparkMax() {
        configuretransitWheelsMotor();
    }

    public void configuretransitWheelsMotor() {
        this.transitWheelsMotor = new SparkMax(TransitWheelsConstants.kTransitWheelsMotorCANID, MotorType.kBrushless);
        SparkMaxConfig transitWheelsConfig = new SparkMaxConfig();
        transitWheelsConfig.inverted(false);
        transitWheelsConfig.idleMode(IdleMode.kBrake);
        transitWheelsConfig.smartCurrentLimit(40);
    }

    @Override
    public void updateInputs(TransitWheelsIOInputs inputs) {
        inputs.transitWheelsRPM = this.transitWheelsMotor.getEncoder().getVelocity();
        inputs.transitWheelsMotorAppliedVolts = this.transitWheelsMotor.getAppliedOutput() * this.transitWheelsMotor.getBusVoltage();
        inputs.transitWheelsMotorCurrentAmps = new double[] {this.transitWheelsMotor.getOutputCurrent()};
    }

    @Override
    public void setDesiredTransitWheelsMotorVoltage(double desiredVoltage) {
        this.transitWheelsMotor.setVoltage(desiredVoltage);
    }

}
