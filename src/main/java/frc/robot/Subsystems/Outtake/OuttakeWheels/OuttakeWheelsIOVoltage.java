package frc.robot.Subsystems.Outtake.OuttakeWheels;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Subsystems.Elevator.ElevatorIO.ElevatorIOInputs;

public class OuttakeWheelsIOVoltage implements OuttakeWheelsIO {
    
    private SparkMax leftWheel;
    private SparkMax rightWheel;

    private double desiredLeftWheel = 0;
    private double desiredRightWheel = 0;

    public OuttakeWheelsIOVoltage() {
        this.leftWheel = new SparkMax(OuttakeConstants.kLeftWheelMotorCANID, MotorType.kBrushless);
        this.rightWheel = new SparkMax(OuttakeConstants.kRightWheelMotorCANID, MotorType.kBrushless);

        configureWheels();
        
        Timer.delay(0.5);
    }

    @Override
    public void setWheels(double leftWheelVoltage, double rightWheelVoltage) {
        desiredLeftWheel = leftWheelVoltage;
        desiredRightWheel = rightWheelVoltage;
        leftWheel.set(leftWheelVoltage);
        rightWheel.set(rightWheelVoltage);
    }

    private void configureWheels() {
        SparkMaxConfig configuration = new SparkMaxConfig();
        configuration.smartCurrentLimit(OuttakeConstants.kWheelSmartCurrentLimit);
        leftWheel.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightWheel.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(OuttakeWheelsIOInputs outtakeIOInputs) {
        outtakeIOInputs.leftWheelRPM = leftWheel.getEncoder().getVelocity();
        outtakeIOInputs.leftWheelDesired = desiredLeftWheel;
        outtakeIOInputs.rightWheelRPM = rightWheel.getEncoder().getVelocity();
        outtakeIOInputs.rightWheelDesired = desiredRightWheel;
        outtakeIOInputs.leftWheelAppliedVolts = leftWheel.getAppliedOutput() * leftWheel.getBusVoltage();
        outtakeIOInputs.rightWheelAppliedVolts = rightWheel.getAppliedOutput() * rightWheel.getBusVoltage();
        outtakeIOInputs.leftWheelCurrentAmps = new double[] {leftWheel.getOutputCurrent()};
        outtakeIOInputs.rightWheelCurrentAmps = new double[] {leftWheel.getOutputCurrent()};
    }

}
