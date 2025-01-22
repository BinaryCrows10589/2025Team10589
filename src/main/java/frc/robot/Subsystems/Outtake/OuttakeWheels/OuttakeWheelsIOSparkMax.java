package frc.robot.Subsystems.Outtake.OuttakeWheels;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class OuttakeWheelsIOSparkMax implements OuttakeWheelsIO {
    
    private SparkMax leftWheel;
    private SparkMax rightWheel;

    private SparkClosedLoopController leftWheelPIDController;
    private SparkClosedLoopController rightWheelPIDController;

    private double desiredLeftWheel = 0;
    private double desiredRightWheel = 0;

    private NetworkTablesTunablePIDConstants leftWheelPIDConstantTuner;
    private NetworkTablesTunablePIDConstants rightWheelPIDConstantTuner;

    public OuttakeWheelsIOSparkMax() {
        this.leftWheel = new SparkMax(OuttakeConstants.kLeftWheelMotorCANID, MotorType.kBrushless);
        this.rightWheel = new SparkMax(OuttakeConstants.kRightWheelMotorCANID, MotorType.kBrushless);

        configureWheels();
        
        Timer.delay(0.5);
    }

    @Override
    public void setWheelPositions(double leftWheelPosition, double rightWheelPosition) {
        desiredLeftWheel = leftWheelPosition;
        desiredRightWheel = leftWheelPosition;

        leftWheelPIDController.setReference(rightWheelPosition, ControlType.kPosition);
        rightWheelPIDController.setReference(rightWheelPosition, ControlType.kPosition);
    }

    @Override
    public void setWheelVoltages(double leftWheelVoltage, double rightWheelVoltage) {
        desiredLeftWheel = leftWheelVoltage;
        desiredRightWheel = rightWheelVoltage;
        leftWheel.setVoltage(leftWheelVoltage);
        rightWheel.setVoltage(rightWheelVoltage);
    }

    private void configureWheels() {
        SparkMaxConfig leftConfiguration = new SparkMaxConfig();
        SparkMaxConfig rightConfiguration = new SparkMaxConfig();

        leftConfiguration.smartCurrentLimit(OuttakeConstants.kWheelSmartCurrentLimit);
        rightConfiguration.smartCurrentLimit(OuttakeConstants.kWheelSmartCurrentLimit);

        leftConfiguration.closedLoop.pid(
            OuttakeConstants.kLeftWheelPPIDValue,
            OuttakeConstants.kLeftWheelIPIDValue,
            OuttakeConstants.kLeftWheelDPIDValue
        );
        rightConfiguration.closedLoop.pid(
            OuttakeConstants.kRightWheelPPIDValue,
            OuttakeConstants.kRightWheelIPIDValue,
            OuttakeConstants.kRightWheelDPIDValue
        );

        this.leftWheelPIDConstantTuner = new NetworkTablesTunablePIDConstants("/Outtake/Wheels/Left", 
        OuttakeConstants.kLeftWheelPPIDValue,
        OuttakeConstants.kLeftWheelIPIDValue,
        OuttakeConstants.kLeftWheelDPIDValue,
        0);
        this.rightWheelPIDConstantTuner = new NetworkTablesTunablePIDConstants("/Outtake/Wheels/Right", 
        OuttakeConstants.kRightWheelPPIDValue,
        OuttakeConstants.kRightWheelIPIDValue,
        OuttakeConstants.kRightWheelDPIDValue,
        0);
        
        leftWheel.configure(leftConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightWheel.configure(rightConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        leftWheelPIDController = leftWheel.getClosedLoopController();
        rightWheelPIDController = rightWheel.getClosedLoopController();

        
    
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
        updatePIDValuesFromNetworkTables();
    }

    private void updatePIDValuesFromNetworkTables() {
        double[] leftWheelPIDValues = this.leftWheelPIDConstantTuner.getUpdatedPIDConstants();
        if(this.leftWheelPIDConstantTuner.hasAnyPIDValueChanged()) {
            SparkMaxConfig newDrivePIDConfigs = new SparkMaxConfig();
            newDrivePIDConfigs.closedLoop.pid(
                leftWheelPIDValues[0], 
                leftWheelPIDValues[1], 
                leftWheelPIDValues[2]);
            this.leftWheel.configure(newDrivePIDConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        double[] rightWheelPIDValues = this.rightWheelPIDConstantTuner.getUpdatedPIDConstants();
        if(this.rightWheelPIDConstantTuner.hasAnyPIDValueChanged()) {
            SparkMaxConfig newDrivePIDConfigs = new SparkMaxConfig();
            newDrivePIDConfigs.closedLoop.pid(
                rightWheelPIDValues[0], 
                rightWheelPIDValues[1], 
                rightWheelPIDValues[2]);
            this.rightWheel.configure(newDrivePIDConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

}
