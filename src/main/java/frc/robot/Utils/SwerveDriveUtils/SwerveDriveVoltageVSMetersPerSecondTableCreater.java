package frc.robot.Utils.SwerveDriveUtils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.MechanismConstants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Utils.CommandUtils.Wait;

public class SwerveDriveVoltageVSMetersPerSecondTableCreater {

    private double[][] voltageVSMetersPerSecondTable = null;
    private Wait waitForNextVoltageTableCheck;
    private double voltageTableCurrentVoltage;
    private int currentIndex = 0;
    private boolean finished = false;

    public void createVoltageVSMetersPerSecondTable(String moduleName, double maxVolts, double voltageSpacing,
        double timeBetweenVoltChecks, double moduleSpeedMetersPerSecond, Consumer<Double> setVoltageMethod) {
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
    }

    private void logTable(String moduleName) {
        this.finished = true;
        if(!moduleName.equals(SwerveDriveConstants.kFrontLeftModuleName)) {
            return;
        }
        String table = "VoltageVSMPS";
        for(int i = 0; i < voltageVSMetersPerSecondTable[0].length; i++) {
            table +=  ":" + (voltageVSMetersPerSecondTable[1][i] + ", " + voltageVSMetersPerSecondTable[0][i]);
        }

        String logMessage = table.substring(13);
        String directoryName = "logs";
        String fileName = "log.csv";

        // Ensure the directory exists
        File directory = new File(directoryName);
        if (!directory.exists()) {
            directory.mkdirs();
        }

        // Define the file path
        File logFile = new File(directory, fileName);

        // Write to the file
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(logFile))) {
            writer.write(logMessage);
            System.out.println("Log written to: " + logFile.getAbsolutePath());
        } catch (IOException e) {
            e.printStackTrace();
        }
        //Logger.recordOutput(moduleName + " VoltageVSMPSTable: ", table);
    }
}
