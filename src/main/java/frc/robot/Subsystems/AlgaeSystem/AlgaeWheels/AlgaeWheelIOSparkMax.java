package frc.robot.Subsystems.AlgaeSystem.AlgaeWheels;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Constants.MechanismConstants.AlgaeWheelConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class AlgaeWheelIOSparkMax implements AlgaeWheelIO {
    
    private SparkMax wheelMotor;

    private SparkClosedLoopController wheelPIDController;

    private double desiredWheelVoltage = 0;

    private double desiredWheelPosition = 0;

    private NetworkTablesTunablePIDConstants wheelPIDConstantTuner;

    public AlgaeWheelIOSparkMax() {
        this.wheelMotor = new SparkMax(AlgaeWheelConstants.kWheelMotorCANID, MotorType.kBrushless);

        configureWheels();
        
        Timer.delay(0.5);
    }

    @Override
    public void setWheelPosition(double desiredPosition) {
        desiredWheelPosition = desiredPosition;

        wheelPIDController.setReference(desiredWheelPosition, ControlType.kPosition);
    }

    @Override
    public void setWheelPositionRelative(double desiredPosition) {
        desiredWheelPosition += desiredPosition;

        wheelPIDController.setReference(desiredWheelPosition, ControlType.kPosition);
    }

    @Override
    public void setWheelVoltage(double voltage) {
        desiredWheelVoltage = voltage;
        wheelMotor.setVoltage(desiredWheelVoltage);
    }

    private void configureWheels() {
        SparkMaxConfig wheelConfiguration = new SparkMaxConfig();

        wheelConfiguration.smartCurrentLimit(AlgaeWheelConstants.kWheelSmartCurrentLimit);

        wheelConfiguration.closedLoop.pid(
            AlgaeWheelConstants.kWheelPPIDValue,
            AlgaeWheelConstants.kWheelIPIDValue,
            AlgaeWheelConstants.kWheelDPIDValue
        );

        this.wheelPIDConstantTuner = new NetworkTablesTunablePIDConstants("/Outtake/Wheels/Left", 
        AlgaeWheelConstants.kWheelPPIDValue,
        AlgaeWheelConstants.kWheelIPIDValue,
        AlgaeWheelConstants.kWheelDPIDValue,
        0);

        wheelConfiguration.idleMode(IdleMode.kBrake);
        
        wheelMotor.configure(wheelConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        wheelPIDController = wheelMotor.getClosedLoopController();

        
    
    }

    @Override
    public void updateInputs(AlgaeWheelIOInputs algaeIOInputs) {
        algaeIOInputs.wheelRPM = wheelMotor.getEncoder().getVelocity();
        algaeIOInputs.wheelDesiredVoltage = desiredWheelVoltage;
        algaeIOInputs.wheelDesiredPosition = desiredWheelPosition;
        algaeIOInputs.wheelAppliedVolts = wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage();
        algaeIOInputs.wheelCurrentAmps = new double[] {wheelMotor.getOutputCurrent()};
        updatePIDValuesFromNetworkTables();
    }

    private void updatePIDValuesFromNetworkTables() {
        wheelPIDConstantTuner.updatePIDValuesFromNetworkTables(wheelMotor);
    }

}
