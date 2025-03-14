package frc.robot.Subsystems.Outtake.OuttakeWheels;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class OuttakeWheelsIOSparkMax implements OuttakeWheelsIO {
    
    private SparkMax rightWheel;

    private SparkClosedLoopController rightWheelPIDController;

    private double desiredRightWheelVoltage = 0;

    private double desiredRightWheelPosition = 0;

    private NetworkTablesTunablePIDConstants rightWheelPIDConstantTuner;

    public OuttakeWheelsIOSparkMax() {
        this.rightWheel = new SparkMax(OuttakeConstants.kRightWheelMotorCANID, MotorType.kBrushless);

        configureWheels();
        
        Timer.delay(0.1);
    }

    @Override
    public void setWheelPositions(double rightWheelPosition) {
        desiredRightWheelPosition = rightWheelPosition;

        rightWheelPIDController.setReference(desiredRightWheelPosition, ControlType.kPosition);
    }

    @Override
    public void setWheelPositionsRelative(double rightWheelPosition) {
        desiredRightWheelPosition += rightWheelPosition;

        rightWheelPIDController.setReference(desiredRightWheelPosition, ControlType.kPosition);
    }

    @Override
    public void setWheelVoltages(double rightWheelVoltage) {
        desiredRightWheelVoltage = rightWheelVoltage;
        rightWheel.setVoltage(desiredRightWheelVoltage);
    }

    private void configureWheels() {
        SparkMaxConfig rightConfiguration = new SparkMaxConfig();

        rightConfiguration.smartCurrentLimit(OuttakeConstants.kWheelSmartCurrentLimit);
        rightConfiguration.inverted(true);

        rightConfiguration.closedLoop.pid(
            OuttakeConstants.kRightWheelPPIDValue,
            OuttakeConstants.kRightWheelIPIDValue,
            OuttakeConstants.kRightWheelDPIDValue
        );
     
        this.rightWheelPIDConstantTuner = new NetworkTablesTunablePIDConstants("/Outtake/Wheels/Right", 
        OuttakeConstants.kRightWheelPPIDValue,
        OuttakeConstants.kRightWheelIPIDValue,
        OuttakeConstants.kRightWheelDPIDValue,
        0);

        rightConfiguration.idleMode(IdleMode.kBrake);
        rightWheel.configure(rightConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightWheelPIDController = rightWheel.getClosedLoopController();
    
    }

    @Override
    public void updateInputs(OuttakeWheelsIOInputs outtakeIOInputs) {
        outtakeIOInputs.rightWheelRPM = rightWheel.getEncoder().getVelocity();
        outtakeIOInputs.rightWheelDesiredVoltage = desiredRightWheelVoltage;
        outtakeIOInputs.rightWheelDesiredPosition = desiredRightWheelPosition;
        outtakeIOInputs.rightWheelAppliedVolts = rightWheel.getAppliedOutput() * rightWheel.getBusVoltage();
        outtakeIOInputs.rightWheelCurrentAmps = new double[] {rightWheel.getOutputCurrent()};
        updatePIDValuesFromNetworkTables();
    }

    private void updatePIDValuesFromNetworkTables() {
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
