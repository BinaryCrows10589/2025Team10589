package frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils;

public class NetworkTablesTunablePIDConstants {

    private NetworkTablesChangableValue changablePValue;
    private NetworkTablesChangableValue changableIValue;
    private NetworkTablesChangableValue changableDValue;
    private NetworkTablesChangableValue changableFFValue;


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

    /**
     * The values of all four PID Constant as an array of doubles
     * @return Double[]-{PValue, IValue, DValue, FFValue} The values of the PID as an array
     */
    public double[] getUpdatedPIDConstants() {
        double[] arrayOfPIDValues = {
            (double)this.changablePValue.getChangableValueOnNetworkTables(),
            (double)this.changableIValue.getChangableValueOnNetworkTables(),
            (double)this.changableDValue.getChangableValueOnNetworkTables(),
            (double)this.changableFFValue.getChangableValueOnNetworkTables()
        };

        return arrayOfPIDValues;
    }

    public boolean hasAnyPIDValueChanged() {
        return this.changablePValue.hasChangableValueChanged() || this.changableIValue.hasChangableValueChanged() ||
            this.changableDValue.hasChangableValueChanged() || this.changableFFValue.hasChangableValueChanged();
    }

}