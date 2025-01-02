// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Subsystems.SwerveDrive.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

    /**
     * Contains the inputs to the swerve module that will be logged.
     */
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double driveMotorRotations = 0.0;
        public double driveMotorRPM = 0.0;
        public double driveMotorSpeedMetersPerSecond = 0.0;
        public double driveMotorDistanceMeters = 0.0;
        public double driveMotorAppliedVolts = 0.0;
        public double[] driveMotorCurrentAmps = new double[] {};

        public double turnMotorAbsolutePositionRotations = 0.0;
        public double turnMotorRelitivePositionRotations = 0.0;
        public double wheelAngleRelitivePositionRotations = 0.0;
        public double turnMotorRPM = 0.0;
        public double turnMotorAppliedVolts = 0.0;
        public double[] turnMotorCurrentAmps = new double[] {};
    }

    /**
     * Updates all loggable inputes
     * @param inputs SwerveModuleIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    /**
     * Sets the desired voltage of the module
     * @param desiredVoltage Double: The desired module voltage
     */
    public default void setDesiredModuleDriveVoltage(double desiredVoltage) {

    }

    /**
     * Sets the desired RPM of the module
     * @param desiredRPM Double: The desired RPM for the module
     */
    public default void setDesiredModuleVelocityRPM(double desiredRPM) {}

    /**
     * Sets the desired angle of the modules wheel
     * @param desiredModuleAngle Rotation2d: The desired angle of the modules wheel
     */
    public default void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {}
}
