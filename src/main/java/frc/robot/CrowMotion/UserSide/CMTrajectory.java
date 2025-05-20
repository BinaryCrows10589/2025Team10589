package frc.robot.CrowMotion.UserSide;

import java.awt.datatransfer.Transferable;
import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPoint;

import frc.robot.CrowMotion.Library.CMPathGenResult;
import frc.robot.CrowMotion.Library.CMPathGenerator;
import frc.robot.CrowMotion.Library.CMPathPoint;
import frc.robot.CrowMotion.UserSide.CMRotation.RotationDirrection;

public class CMTrajectory {
        
    private String pathName;
    private double maxDesiredTranslationalVelocity;
    private double maxDesiredRotationalVelocity;
    private TrajectoryPriority trajectoryPriority;
    private double endVelocity;
    private boolean shouldStopAtEnd;
    private double[] positionTolorence;
    private double maxTime;
    private CompletableFuture<CMPathGenResult> futurePath;
    
    private CMPathPoint[] path = null;
    private double[] endRobotState;
    private double endTime = -1;
    private boolean isComplete;

    private int lastGoalPoint;
    private int goalPoint;

    public static enum TrajectoryPriority {
        PREFER_ROTATION,
        PREFER_TRANSLATION,
        SPLIT_PROPORTIONALLY
    }
    
    public CMTrajectory(String pathName, CMAutonPoint[] controlPoints, double initialRotation,
        CMRotation[] rotations, CMEvent[] events,
        double maxDesiredTranslationalVelocity,
        double maxDesiredRotationalVelocity,
        TrajectoryPriority trajectoryPriority,
        double endVelocity,
        boolean shouldStopAtEnd, double[] positionTolorence, double maxTime) {
        this.pathName = pathName;
        this.maxDesiredTranslationalVelocity = maxDesiredTranslationalVelocity;
        this.maxDesiredRotationalVelocity = maxDesiredRotationalVelocity;
        this.trajectoryPriority = trajectoryPriority;
        this.endVelocity = endVelocity;
        this.shouldStopAtEnd = shouldStopAtEnd;
        this.positionTolorence = positionTolorence;
        this.maxTime = maxTime;

        assert controlPoints.length >= 1 : "For" + pathName + "CrowMotion paths need at least one control point";
        this.futurePath = CMPathGenerator.generateCMPathAsync("TestBezier",
                controlPoints, initialRotation, rotations, events);
        CMAutonPoint lastPoint = controlPoints[controlPoints.length-1];
        double endRotation = rotations.length == 0 ? initialRotation : rotations[rotations.length-1].getAngleDegrees();
        this.endRobotState = new double[] {lastPoint.getX(), lastPoint.getY(), endRotation};
    }

    public void runTrejectoryFrame() {
        double[] robotPosition = CMConfig.getRobotPositionMetersAndDegrees();
        this.isComplete = shouldEnd(robotPosition);
        if(this.isComplete) {
            // Clean Up
        } else {
            loadPath();
            //double[] velocities = calculateVelocities()
            //this.goalPoint = selectGoalPoint(velocities);
            // Calculate goal point bassed purly on translational velocities
            // Calculate goal desired rotational velocity
            // Desaturate
            // "Walk back" point by point until a point is found that would require the correct sum of rotational and translational velocity
            
        }
        
        
    }

    public boolean isCompleted() {
        return this.isComplete;
    }

    private boolean shouldEnd(double[] robotPosition) {
        boolean inXTolorence = Math.abs(robotPosition[0] - this.endRobotState[0]) < this.positionTolorence[0];
        boolean inYTolorence = Math.abs(robotPosition[1] - this.endRobotState[1]) < this.positionTolorence[1];
        boolean inRotationTolorence = Math.abs(robotPosition[2] - this.endRobotState[2]) < this.positionTolorence[2];
        boolean hasTimeElasped = endTime != -1 && System.currentTimeMillis() >= endTime;
        return inXTolorence && inYTolorence && inRotationTolorence && hasTimeElasped;
    }

    private void loadPath() {
        if(path == null && futurePath.isDone()) {
            CMPathGenResult result = futurePath.getNow(new CMPathGenResult(null, null));
            path = result.path;
            Logger.recordOutput("CrowMotion/" + pathName + "/Trajectory", result.loggingPoints);
            this.endTime = System.currentTimeMillis() + this.maxTime;
        }
    }


    private double[] desaturateVelocitiesMagnitudes(double[] velocitiesMPSAndDPSMagnitudes) {
        double translationalVelocity = velocitiesMPSAndDPSMagnitudes[0];
        double rotationalTranslationVelocity = CMConfig.getWheelCircumference() * (velocitiesMPSAndDPSMagnitudes[1]/360);
        double requiredWheelVelocity = translationalVelocity + rotationalTranslationVelocity;
        double maxWheelVelocity = CMConfig.getRobotProfile().getMaxPossibleAverageSwerveModuleMPS();
        if(velocitiesMPSAndDPSMagnitudes[0] == 0) {
            return new double[] {maxWheelVelocity, 0};
        }
        if(requiredWheelVelocity < maxWheelVelocity) {
            return velocitiesMPSAndDPSMagnitudes;
        } else {
            if(this.trajectoryPriority == TrajectoryPriority.SPLIT_PROPORTIONALLY) {
                double percentTranslational = translationalVelocity / requiredWheelVelocity;
                double percentRotational = rotationalTranslationVelocity / requiredWheelVelocity;

                return new double[] {percentTranslational * maxWheelVelocity, ((percentRotational * maxWheelVelocity)/(velocitiesMPSAndDPSMagnitudes[1]/360))};
            } else if(this.trajectoryPriority == TrajectoryPriority.PREFER_ROTATION) {
                if(rotationalTranslationVelocity < maxWheelVelocity) {
                    double newTranslationalVelocity = maxWheelVelocity - rotationalTranslationVelocity;
                    return new double[] {newTranslationalVelocity, velocitiesMPSAndDPSMagnitudes[1]};
                } else {
                    return new double[] {0, maxWheelVelocity/(velocitiesMPSAndDPSMagnitudes[1]/360)};
                }
            } else{
                if(translationalVelocity < maxWheelVelocity) {
                    double newRotationalVelocity = maxWheelVelocity - translationalVelocity;
                    return new double[] {translationalVelocity, newRotationalVelocity};
                } else {
                    return new double[] {maxWheelVelocity, 0};
                }
            }
        }
    }

}
