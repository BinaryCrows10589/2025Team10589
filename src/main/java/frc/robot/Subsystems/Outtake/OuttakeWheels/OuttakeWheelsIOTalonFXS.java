package frc.robot.Subsystems.Outtake.OuttakeWheels;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MechanismConstants.OuttakeConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class OuttakeWheelsIOTalonFXS implements OuttakeWheelsIO {
    
    private TalonFXS rightWheel;

    private double desiredRightWheelVoltage = 0;
    private double desiredRightWheelPosition = 0;

    private NetworkTablesTunablePIDConstants rightWheelPIDConstantTuner;

    public OuttakeWheelsIOTalonFXS() {
        this.rightWheel = new TalonFXS(OuttakeConstants.kRightWheelMotorCANID);

        configureWheels();
        
        Timer.delay(0.1);
    }

    @Override
    public void setWheelPositions(double rightWheelPosition) {
        desiredRightWheelPosition = rightWheelPosition;
        rightWheel.setControl(new PositionVoltage(desiredRightWheelPosition));
    }

    @Override
    public void setWheelPositionsRelative(double rightWheelPosition) {
        desiredRightWheelPosition += rightWheelPosition;
        rightWheel.setControl(new PositionVoltage(desiredRightWheelPosition));
    }

    @Override
    public void setWheelVoltages(double rightWheelVoltage) {
        desiredRightWheelVoltage = rightWheelVoltage;
        rightWheel.setVoltage(desiredRightWheelVoltage);
    }

    private void configureWheels() {
        TalonFXSConfiguration rightConfiguration = new TalonFXSConfiguration();

        rightConfiguration.Voltage.PeakForwardVoltage = OuttakeConstants.kMaxWheelVoltage;
        rightConfiguration.Voltage.PeakReverseVoltage = -OuttakeConstants.kMaxWheelVoltage;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs outtakeWheelPositioConfigs = new Slot0Configs();

        outtakeWheelPositioConfigs.kP = OuttakeConstants.kRightWheelPPIDValue;
        outtakeWheelPositioConfigs.kP = OuttakeConstants.kRightWheelIPIDValue;
        outtakeWheelPositioConfigs.kP = OuttakeConstants.kRightWheelDPIDValue;
     
        this.rightWheelPIDConstantTuner = new NetworkTablesTunablePIDConstants("/Outtake/Wheels/Right", 
        OuttakeConstants.kRightWheelPPIDValue,
        OuttakeConstants.kRightWheelIPIDValue,
        OuttakeConstants.kRightWheelDPIDValue,
        0);

        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightWheel.getConfigurator().apply(rightConfiguration);
        rightWheel.getConfigurator().apply(outtakeWheelPositioConfigs);
    }

    @Override
    public void updateInputs(OuttakeWheelsIOInputs outtakeIOInputs) {
        outtakeIOInputs.rightWheelRPM = rightWheel.getVelocity().getValueAsDouble();
        outtakeIOInputs.rightWheelDesiredVoltage = desiredRightWheelVoltage;
        outtakeIOInputs.rightWheelDesiredPosition = desiredRightWheelPosition;
        outtakeIOInputs.rightWheelAppliedVolts = rightWheel.getMotorVoltage().getValueAsDouble();
        outtakeIOInputs.rightWheelCurrentAmps = new double[] {rightWheel.getSupplyCurrent().getValueAsDouble()};
        this.rightWheelPIDConstantTuner.updatePIDValuesFromNetworkTables(rightWheel);
    }



}
