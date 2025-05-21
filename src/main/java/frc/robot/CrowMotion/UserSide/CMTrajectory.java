package frc.robot.CrowMotion.UserSide;

import java.awt.datatransfer.Transferable;
import java.awt.geom.Point2D;
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
    private double desiredTranslationalAcceleration;
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
    private boolean isComplete = false;
    private double estimatedTravelDistenceMeters = 0;
    private double estimatedRotationDegrees = 0;
    private double lastFrameStartTime = 0;
    private double averageFrameTime = .02;
    private double lastTranslationErrorRatio = 0;
    private double lastRotationErrorRatio = 0;
    private double[] lastRobotPosition;


    private int lastGoalPoint = -1;
    private int goalPoint = -1;
    
    public static enum TrajectoryPriority {
        PREFER_ROTATION,
        PREFER_TRANSLATION,
        SPLIT_PROPORTIONALLY
    }
        
        public CMTrajectory(String pathName, CMAutonPoint[] controlPoints, double initialRotation,
            CMRotation[] rotations, CMEvent[] events,
            double maxDesiredTranslationalVelocity,
            double desiredTranslationalAcceleration,
            double maxDesiredRotationalVelocity,
            TrajectoryPriority trajectoryPriority,
            double endVelocity,
            boolean shouldStopAtEnd, double[] positionTolorence, double maxTime) {
            this.pathName = pathName;
            this.maxDesiredTranslationalVelocity = maxDesiredTranslationalVelocity;
            this.desiredTranslationalAcceleration = desiredTranslationalAcceleration;
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
            if(this.lastFrameStartTime == -1) {
                this.lastFrameStartTime = (System.currentTimeMillis() / 1000.0) - (1.0 / 50);
            }
            // Arbitraryl large frame time error. 
            this.averageFrameTime = (averageFrameTime + (System.currentTimeMillis() - this.lastFrameStartTime)/1000) / 2;
            this.lastFrameStartTime = System.currentTimeMillis();
        
            double[] robotPosition = CMConfig.getRobotPositionMetersAndDegrees();
            this.isComplete = shouldEnd(robotPosition);
            if(this.isComplete) {
                this.lastGoalPoint = -1;
                this.goalPoint = -1;
                this.endTime = -1;
                this.lastFrameStartTime = -1;
                if(this.shouldStopAtEnd) {
                    CMConfig.setRobotVelocityMPSandDPS(0, 0, 0);
                }
            } else {
                loadPath();
                if(this.path != null && this.shouldStopAtEnd) {
                    double translationErrorRatio = 1;
                    double rotationErrorRatio = 1;
                    if(this.lastGoalPoint != -1) {
                        double travelDistence = calculateMagnitude(lastRobotPosition[0] - robotPosition[0], lastRobotPosition[1] - robotPosition[1]);
                        double rotationTravel = Math.abs(lastRobotPosition[2] - robotPosition[2]);
                        translationErrorRatio = (this.estimatedTravelDistenceMeters / travelDistence) * lastTranslationErrorRatio;;
                        rotationErrorRatio = (this.estimatedRotationDegrees / rotationTravel) * lastRotationErrorRatio;;
                    } else {
                        this.lastGoalPoint = 0;
                    }
                    double translationVelocityMagnatude = calculateTranslationalVelocity();
                    double travelDistence = translationVelocityMagnatude * this.averageFrameTime;
                    double goalDistence = travelDistence;
                    Logger.recordOutput("CrowMotion/AverageFrameTime", averageFrameTime);
                    for(int i = this.lastGoalPoint; i < path.length; i++) {
                        Point2D.Double point = path[i].getTranslationalPoint();
                        double distence = calculateMagnitude(point.x - robotPosition[0] , point.y - robotPosition[1]);
                        Logger.recordOutput("CrowMotion/Distnece", distence);
                        Logger.recordOutput("CrowMotion/TravelDistence", travelDistence);
                        Logger.recordOutput("CrowMotion/TravelDistenceCheck", distence >= travelDistence);

                        if(distence >= travelDistence) {
                            this.lastGoalPoint = this.goalPoint;
                            this.goalPoint = i;
                            goalDistence = travelDistence;
                            break;
                        }
                    } 
                    Logger.recordOutput("CrowMotion/GoalPoint", goalPoint);
                    this.shouldStopAtEnd = false;
                    /* 
                    double rotationalVelocity = Math.min(this.maxDesiredRotationalVelocity ,
                        (path[this.goalPoint].getDesiredRotation() - robotPosition[2]) *
                        (travelDistence / goalDistence) / this.averageFrameTime);
                    double[] desaturatedSpeeds = desaturateVelocitiesMagnitudes(new double[] {translationVelocityMagnatude, Math.abs(rotationalVelocity)});
                    translationVelocityMagnatude = desaturatedSpeeds[0];
                    rotationalVelocity = Math.signum(rotationalVelocity) * desaturatedSpeeds[1];
                    this.estimatedTravelDistenceMeters = translationVelocityMagnatude * this.averageFrameTime;
                    this.estimatedRotationDegrees = rotationalVelocity * this.averageFrameTime;
                    // Do not apply velocity limits as these are derivited from limited desired values and are corrections to better match desired
                    translationVelocityMagnatude *= translationErrorRatio; 
                    rotationalVelocity *= rotationErrorRatio;
                    Point2D.Double goalTranslatinPoint = path[goalPoint].getTranslationalPoint();
                    double[] translationVelocities = calculateXAndYComponenteVelocities(goalTranslatinPoint.x - robotPosition[0],
                        goalTranslatinPoint.y - robotPosition[1], translationVelocityMagnatude);
                            */
                    this.lastRobotPosition = robotPosition;
                    //CMConfig.setRobotVelocityMPSandDPS(translationVelocities[0], translationVelocities[1], rotationalVelocity); 
                 
                } 
                
        }
    }

    public boolean isCompleted() {
        if(isComplete) {
            this.lastGoalPoint = -1;
            this.goalPoint = -1;
            this.endTime = -1;
        }
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

    private double calculateTranslationalVelocity() {
        double[] currentVelocities = CMConfig.getRobotVelocityMPSandDPS();
        double[] robotPosition = CMConfig.getRobotPositionMetersAndDegrees();
        double currentTranslationMagnitude = calculateMagnitude(currentVelocities[0], currentVelocities[1]);
        Point2D.Double endPoint = this.path[path.length-1].getTranslationalPoint();
        double currentDistanceFromEndPoint = calculateMagnitude(robotPosition[0] - endPoint.x, robotPosition[1] - endPoint.y);
        double timeToSlowDown = (currentTranslationMagnitude - this.endVelocity) / this.desiredTranslationalAcceleration;
        double minTravelTime = currentDistanceFromEndPoint / ((currentTranslationMagnitude + this.endVelocity) / 2);
        if(minTravelTime > timeToSlowDown) {
            double endVelocityIfAccelerate = currentTranslationMagnitude + (this.desiredTranslationalAcceleration * this.averageFrameTime);
            return Math.min(this.maxDesiredTranslationalVelocity, endVelocityIfAccelerate);
        } else {
            double endVelocityIfDeccelerate = currentTranslationMagnitude - (this.desiredTranslationalAcceleration * this.averageFrameTime);
            return Math.max(this.endVelocity, endVelocityIfDeccelerate);
        }
    }

    private double[] calculateXAndYComponenteVelocities(double deltaX, double deltaY, double translationalVelocityMagnitude) {
        double length = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        if (length == 0) return new double[] {0, 0}; 
        double dx = (translationalVelocityMagnitude * deltaX) / length;
        double dy = (translationalVelocityMagnitude * deltaY) / length;
        return new double[] {dx, dy};
    }

    private double calculateMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
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
