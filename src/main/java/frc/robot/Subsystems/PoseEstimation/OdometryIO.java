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

package frc.robot.Subsystems.PoseEstimation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

import org.littletonrobotics.junction.AutoLog;

public interface OdometryIO {

    /**
     * Contains the inputs to the swerve module that will be logged. They are also used in the SwerveModule class
     */
    @AutoLog
    public static class OdometryIOInputs {
        public Pose2d robotPosition = new Pose2d();
    }

    /**
     * Updates all loggable inputes
     * @param inputs OdometryIOInputs: The inputes that will be logged. 
     */
    public default void updateInputs(OdometryIOInputs inputs) {}

    /**
     * Sets the robots position
     * @param newRobotPose AutonPoint: The new robot pose that should be set. Will be mirrored based on alliance. 
     */
    public default void setRobotPose(AutonPoint newRobotPose) {}

    /**
     * Sets the robots position
     * @param newRobotPose Pose2d: The new robot pose that should be set.
     */
    public default void setRobotPose(Pose2d newRobotPose) {}

    /**
     * Sets the robots position to be at the 0x, 0y, 0 degrees
     * @return Pose2d: The estimated position of the robot. 
     */
    public default void resetRobotPose() {}

    /**
     * Sets the orgin position for april tags based on the current alliance
     */
    public default void updateAlliance() {}

}