package frc.robot.Utils.SwerveDriveUtils;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.Utils.CommandUtils.Wait;

public class SwerveDriveVoltageVSMetersPerSecondTableCreater {

    private double[][] voltageVSMetersPerSecondTable = null;
    private Wait waitForNextVoltageTableCheck;
    private double voltageTableCurrentVoltage;
    private int currentIndex = 0;
    private boolean finished = false;

    private String table;

    public void createVoltageVSMetersPerSecondTable(String moduleName, double maxVolts, double voltageSpacing,
        double timeBetweenVoltChecks, double moduleSpeedMetersPerSecond, Consumer<Double> setVoltageMethod) {
        table = "VoltageVSMPS";
        if(voltageVSMetersPerSecondTable == null) {
            int rowsInTable = (int)(maxVolts / voltageSpacing);
            voltageVSMetersPerSecondTable = new double[2][rowsInTable];
            waitForNextVoltageTableCheck = new Wait(timeBetweenVoltChecks);
            waitForNextVoltageTableCheck.startTimer();
        } else {
            if(currentIndex < voltageVSMetersPerSecondTable[0].length) {
                if(waitForNextVoltageTableCheck.hasTimePassed()) {
                    voltageVSMetersPerSecondTable[0][currentIndex] = voltageTableCurrentVoltage;
                    voltageVSMetersPerSecondTable[1][currentIndex] = moduleSpeedMetersPerSecond;

                    currentIndex++;
                    setVoltageMethod.accept(voltageTableCurrentVoltage);
                    voltageTableCurrentVoltage += voltageSpacing;
                    waitForNextVoltageTableCheck.startTimer();
                }
            } else {
                if(!finished)
                    logTable(moduleName);
            }
        }    
        System.out.println(table);       
    }

    private void logTable(String moduleName) {
        finished = true;
        String tableInCSVFormat = moduleName +" VoltageVSMetersPerSecondData:\n";
        for(int i = 0; i < voltageVSMetersPerSecondTable[0].length; i++) {
            table = table.concat("\n" + voltageVSMetersPerSecondTable[0][i] + ", " + voltageVSMetersPerSecondTable[1][i]);
        }
        System.out.println(tableInCSVFormat);
        //Logger.recordOutput(moduleName + " VoltageVSMPSTable: ", voltageVSMetersPerSecondTable[0][0]);
    }
}
