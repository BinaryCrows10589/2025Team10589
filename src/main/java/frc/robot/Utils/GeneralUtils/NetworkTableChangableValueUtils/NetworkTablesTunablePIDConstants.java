package frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class NetworkTablesTunablePIDConstants {

    private NetworkTablesChangableValue changablePValue;
    private NetworkTablesChangableValue changableIValue;
    private NetworkTablesChangableValue changableDValue;
    private NetworkTablesChangableValue changableFFValue;

    // Specific to certain implementations of PID
    private NetworkTablesChangableValue changableGValue; // Gravity
    private NetworkTablesChangableValue changableSValue; // Static friction
    private NetworkTablesChangableValue changableVValue; // Velocity
    private NetworkTablesChangableValue changableAValue; // Acceleration
    private NetworkTablesChangableValue changableCruseVelocityValue;
    private NetworkTablesChangableValue changableAccelerationValue;
    private NetworkTablesChangableValue changableJerkValue;
    
    private boolean isUsingAdvancedPID = false; // Whether the variables above will be used


    /**
     * WORNING!!! THIS WILL NOT WORK DURING MATCHES AT COMPETION.
     * ALL PID VALUES TUNED HERE SHOULD BE ADDED TO THE RELEVENT CONSTANT FILE ONCE TUNING IS COMPLETE!!!
     * WORNING!!! VALUES WILL NOT STAY BETWEEN DASHBOARD OR ROBOT REBOOTS/CODE REDEPLOYS.
     * Please record the value somehow before proforming these actions to not loose progress
     * @param networkTablesKey String: The start of the key under which the 
     * values will be added to network tables. e.x Module/DrivePIDValues
     *
     * @param defaultPValue Double: The defualt P PID value
     * @param defaultIValue Double: The defualt I PID value
     * @param defaultDValue Double: The defualt D PID value
     * @param defaultFFValue Double: The defualt FF PID value
     */
    public NetworkTablesTunablePIDConstants(String baseNetworkTablesKey, double defaultPValue,
        double defaultIValue, double defaultDValue, double defaultFFValue) {

        this.changablePValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/PValue", (double)defaultPValue);
        this.changableIValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/IValue", (double)defaultIValue);
        this.changableDValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/DValue", (double)defaultDValue);
        this.changableFFValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/FFValue", (double)defaultFFValue);
    }

    public NetworkTablesTunablePIDConstants(String baseNetworkTablesKey,
    double defaultPValue,
    double defaultIValue,
    double defaultDValue,
    double defaultFFValue,
    double defaultGValue,
    double defaultSValue,
    double defaultVValue,
    double defaultAValue,
    double defaultVelocityValue,
    double defaultAccelerationValue,
    double defaultJerkValue) {
        this.changablePValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/PValue", (double)defaultPValue);
        this.changableIValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/IValue", (double)defaultIValue);
        this.changableDValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/DValue", (double)defaultDValue);
        this.changableFFValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/FFValue", (double)defaultFFValue);
        this.changableGValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/GValue", (double)defaultGValue);
        this.changableSValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/SValue", (double)defaultSValue);
        this.changableVValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/VValue", (double)defaultVValue);
        this.changableAValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/AValue", (double)defaultAValue);
        this.changableCruseVelocityValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/CruseVelocityValue", (double) defaultVelocityValue);
        this.changableAccelerationValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/AccelerationValue", (double)defaultAccelerationValue);
        this.changableJerkValue = new NetworkTablesChangableValue(baseNetworkTablesKey + "/JerkValue", (double)defaultJerkValue);
        isUsingAdvancedPID = true;
    }

    
    

    /**
     * The values of all four PID Constant as an array of doubles
     * @return Double[]-{PValue, IValue, DValue, FFValue, (GValue, SValue)} The values of the PID as an array
     */
    public double[] getUpdatedPIDConstants() {
        double[] arrayOfPIDValues;

        if (isUsingAdvancedPID) {

            arrayOfPIDValues = new double[] {
                (double)this.changablePValue.getChangableValueOnNetworkTables(),
                (double)this.changableIValue.getChangableValueOnNetworkTables(),
                (double)this.changableDValue.getChangableValueOnNetworkTables(),
                (double)this.changableFFValue.getChangableValueOnNetworkTables(),
                (double)this.changableGValue.getChangableValueOnNetworkTables(),
                (double)this.changableSValue.getChangableValueOnNetworkTables(),
                (double)this.changableVValue.getChangableValueOnNetworkTables(),
                (double)this.changableAValue.getChangableValueOnNetworkTables(),
                (double)this.changableCruseVelocityValue.getChangableValueOnNetworkTables(),
                (double)this.changableAccelerationValue.getChangableValueOnNetworkTables(),
                (double)this.changableJerkValue.getChangableValueOnNetworkTables()
            };

        } else {

            arrayOfPIDValues = new double[] {
                (double)this.changablePValue.getChangableValueOnNetworkTables(),
                (double)this.changableIValue.getChangableValueOnNetworkTables(),
                (double)this.changableDValue.getChangableValueOnNetworkTables(),
                (double)this.changableFFValue.getChangableValueOnNetworkTables()
            };

        }
        return arrayOfPIDValues;
    }

    public boolean hasAnyPIDValueChanged() {
        boolean valuesChanged =  
        this.changablePValue.hasChangableValueChanged() || 
        this.changableIValue.hasChangableValueChanged() ||
        this.changableDValue.hasChangableValueChanged() || 
        this.changableFFValue.hasChangableValueChanged();
        if (isUsingAdvancedPID) {
            return valuesChanged ||
            this.changableGValue.hasChangableValueChanged() ||
            this.changableSValue.hasChangableValueChanged() ||
            this.changableVValue.hasChangableValueChanged() ||
            this.changableAValue.hasChangableValueChanged() ||
            this.changableCruseVelocityValue.hasChangableValueChanged() ||
            this.changableAccelerationValue.hasChangableValueChanged() ||
            this.changableJerkValue.hasChangableValueChanged();
        } else {
            return valuesChanged;
        }
    }

    public void updatePIDValuesFromNetworkTables(SparkMax motor) {
        if (hasAnyPIDValueChanged()) {
            double[] pidConstants = getUpdatedPIDConstants();
            SparkMaxConfig newConfig = new SparkMaxConfig();
            newConfig.closedLoop.pid(pidConstants[0], pidConstants[1], pidConstants[2]);
            motor.configure(newConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
    public void updatePIDValuesFromNetworkTables(TalonFX motor) {
        if (hasAnyPIDValueChanged()) {
            double[] pidConstants = getUpdatedPIDConstants();
            Slot0Configs newConfig = new Slot0Configs();
            newConfig.kP = pidConstants[0];
            newConfig.kI = pidConstants[1];
            newConfig.kD = pidConstants[2];
            motor.getConfigurator().apply(newConfig);
        }
    }

    public void updatePIDValuesFromNetworkTables(TalonFXS motor) {
        if (hasAnyPIDValueChanged()) {
            double[] pidConstants = getUpdatedPIDConstants();
            Slot0Configs newConfig = new Slot0Configs();
            newConfig.kP = pidConstants[0];
            newConfig.kI = pidConstants[1];
            newConfig.kD = pidConstants[2];
            motor.getConfigurator().apply(newConfig);
        }
    }

}