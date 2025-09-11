package frc.robot.CrowMotion.UserSide;

import java.awt.geom.Point2D;
import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.Logger;

import frc.robot.CrowMotion.Library.CMPathGenResult;
import frc.robot.CrowMotion.Library.CMPathGenerator;
import frc.robot.CrowMotion.Library.CMPathPoint;

public class CMTrajectory {
    private String pathName;
    private double drivebaseCircumference;
    private double maxModuleVelocity;
    private double maxDesiredTranslationalVelocity;
    private double desiredTranslationalAcceleration;
    private double desiredTranslationalDecceleration;
    private TrajectoryPriority trajectoryPriority;
    private double endVelocity;
    private double minStartVelocity;
    private double distanceAtEndVelocity;
    private boolean shouldStopAtEnd;
    private double[] positionTolerance;
    private double lookAheadMultiplier;
    private double rotationSettleTime;
    private double maxTime;

    private CompletableFuture<CMPathGenResult> futurePath;
    private CMPathGenResult pathGenResult = null;
    private CMPathPoint[] path = null;
    private CMRotation[] rotationDeadlines = null;
    private CMEvent[] events = null;
    private double[] endRobotState;

    private long endTime = -1;
    private long rotationSettleEndTime = -1;
    private long frameStartTime = -1;

    private boolean isComplete = false;
    private boolean preventTranslation = false;

    private double averageFrameTime = 0.02;
    private double maxFrameTime = 0;

    private int currentMinPointIndex = 1;
    private int goalPointIndex = 0;
    private CMPathPoint lastMinPointPathPoint = null;
    private CMPathPoint currentMinPointPathPoint = null;
    private CMPathPoint endPoint = null;
    private Point2D.Double lastMinPoint = null;
    private Point2D.Double currentMinPoint = null;
    private Point2D.Double nextMinPoint = null;

    private double lastDesiredVelocity = -1;
    private double velocityDelta = 0;
    private double initialDistanceToEnd = 0;
    private boolean shouldDecelerate = false;
    private boolean firstDecelerationFrame = true;

    private int lastRotationDeadlineIndex = -1;
    private int currentRotationDeadlineIndex = 0;
    private CMRotation rotationDeadline = null;

    private double desiredRotationDegrees = 0;
    private double maxRotationVelocityDegrees = 0;
    private double desiredRotationalAccelerationDegrees = 0;
    private double desiredRotatioanalDecelerationDegrees = 0;
    private int rotationDirection = 0;

    private double rotationCorrectionRangeDegrees = 0;
    private double maxRotationCorrectionVelocityDegrees = 0;
    private double minRotationVelocityToMove = 0;
    private double maxRotationToleranceDegrees = 0;
    private double decelerationBufferDegrees = 0;
    private double rotationDeadlineCompletePercent = 0;

    private double lastDesiredRotationalVelocity = 0;
    private boolean firstRotationDecelerationFrame = true;
    private boolean enteredCorrectionRange = false;
    private boolean shouldDecelerateRotation = false;
    private double initalRotationalVelocity = 0;
    private double initialDegreesToEnd = 0;

    private int lastTriggeredEventIndex = -1;
            
            public static enum TrajectoryPriority {
                PREFER_ROTATION,
                PREFER_TRANSLATION,
                SPLIT_PROPORTIONALLY
            }
        
            public CMTrajectory(
                String pathName,
                CMAutonPoint[] controlPoints,
                CMRotation[] rotations,
                CMEvent[] events,
                double pointsPerMeter,
                double maxDesiredTranslationalVelocity,
                double desiredTranslationalAcceleration,
                double desiredTranslationalDecceleration,
                TrajectoryPriority trajectoryPriority,
                double endVelocity,
                double distanceAtEndVelocity,
                double minStartVelocity,
                boolean shouldStopAtEnd,
                double[] positionTolerance,
                double lookAheadMultiplier,
                double rotationSettleTime,
                double maxTime) {
                    
                this.pathName = pathName;
                this.maxDesiredTranslationalVelocity = maxDesiredTranslationalVelocity;
                this.desiredTranslationalAcceleration = desiredTranslationalAcceleration;
                this.desiredTranslationalDecceleration = desiredTranslationalDecceleration;
                this.trajectoryPriority = trajectoryPriority;
                this.endVelocity = endVelocity;
                this.distanceAtEndVelocity = distanceAtEndVelocity;
                this.minStartVelocity = minStartVelocity;
                this.shouldStopAtEnd = shouldStopAtEnd;
                this.positionTolerance = positionTolerance;
                this.lookAheadMultiplier = lookAheadMultiplier;
                this.maxTime = maxTime;
                this.rotationSettleTime = rotationSettleTime;
                
                if(controlPoints.length == 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " CrowMotion paths need at least one control point");
                }
                if(this.endVelocity > maxDesiredTranslationalVelocity) {
                    throw new ExceptionInInitializerError("For " + pathName + " CrowMotion End Velocities must be less then or equal to max translational velocity");
                }
                if(this.maxTime <= 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Max Time must be greater than 0");
                }
                if(this.lookAheadMultiplier <= 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Look ahead multiplier must be greater than 0");
                }
                if(this.distanceAtEndVelocity < 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Distance at end velocity must be at least 0");
                }
                if(this.minStartVelocity < 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Min start velocity must be at least 0");
                }
                if(this.rotationSettleTime < 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Time at final position must be at least 0");
                }
                if(this.maxDesiredTranslationalVelocity <= 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Max desired translational velocity must be greater then 0");
                }
                if(this.desiredTranslationalAcceleration <= 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Desired translational acceleration must be greater then 0");
                }
                if(this.desiredTranslationalDecceleration <= 0) {
                    throw new ExceptionInInitializerError("For " + pathName + " Desired translational deceleration must be greater then 0");
                }
                this.drivebaseCircumference = CMConfig.getDrivebaseCircumference();
                this.maxModuleVelocity = CMConfig.getRobotProfile().getMaxPossibleAverageSwerveModuleMPS();
        
                this.futurePath = CMPathGenerator.generateCMPathAsync(pathName,
                        controlPoints, rotations, events, pointsPerMeter);
                CMAutonPoint lastPoint = controlPoints[controlPoints.length - 1];
                
                this.endRobotState = new double[] { lastPoint.getX(), lastPoint.getY()};
            }
        
            public void init() {
                endTime = -1;
                rotationSettleEndTime = -1;
                frameStartTime = -1;
        
                isComplete = false;
                preventTranslation = false;
                
                averageFrameTime = 0.02;
                maxFrameTime = 0;
            
                currentMinPointIndex = 1;
                goalPointIndex = 0;
                lastMinPointPathPoint = null;
                currentMinPointPathPoint = null;
                endPoint = null;
                lastMinPoint = null;
                currentMinPoint = null;
                nextMinPoint = null;
            
                lastDesiredVelocity = -1;
                velocityDelta = 0;
                initialDistanceToEnd = 0;
                shouldDecelerate = false;
                firstDecelerationFrame = true;
            
                lastRotationDeadlineIndex = -1;
                currentRotationDeadlineIndex = 0;
                rotationDeadline = null;
            
                desiredRotationDegrees = 0;
                maxRotationVelocityDegrees = 0;
                desiredRotationalAccelerationDegrees = 0;
                desiredRotatioanalDecelerationDegrees = 0;
                rotationDirection = 0;
            
                rotationCorrectionRangeDegrees = 0;
                maxRotationCorrectionVelocityDegrees = 0;
                minRotationVelocityToMove = 0;
                maxRotationToleranceDegrees = 0;
                decelerationBufferDegrees = 0;
                rotationDeadlineCompletePercent = 0;
                
                lastDesiredRotationalVelocity = 0;
                firstRotationDecelerationFrame = true;
                enteredCorrectionRange = false;
                shouldDecelerateRotation = false;
                initalRotationalVelocity = 0;
                initialDegreesToEnd = 0;
            
                lastTriggeredEventIndex = -1;
        
                if(events != null) {
                    for(int i = 0; i < this.events.length; i++) {
                        events[i].setHasBeenTriggered(false);
                    }
                }
            }
        
    public void runTrajectoryFrame() {
        long currentTime = System.currentTimeMillis();
    
        if(frameStartTime != -1) {
            double frameTime = (currentTime - frameStartTime) / 1000.0; 
            if(frameTime > maxFrameTime) {
                maxFrameTime = frameTime;
                Logger.recordOutput("CrowMotion/Debug/MaxFrameTime", maxFrameTime);
            }
            averageFrameTime = (averageFrameTime * .9) + (frameTime * .1); 
            Logger.recordOutput("CrowMotion/Debug/FrameTime", averageFrameTime);
        }
        frameStartTime = currentTime;

        double[] robotPosition = CMConfig.getRobotPositionMetersAndDegrees();
        double robotX =  robotPosition[0];
        double robotY =  robotPosition[1];
        double robotRot =  robotPosition[2];

        this.isComplete = shouldEnd(robotX, robotY, robotRot, currentTime);
        Logger.recordOutput("CrowMotion/" + pathName + "IsComplete", isComplete);
        Logger.recordOutput("CrowMotion/" + pathName + "IsNull", path == null);
        if(!this.isComplete) {
            this.isComplete = shouldEnd(robotX, robotY, robotRot, currentTime);
        }
        if(this.path != null) {
            if(this.isComplete) {
                if(events != null) {
                    for(int i = lastTriggeredEventIndex+1; i < events.length; i++) {
                        CMEvent event = events[i];
                        if(!event.getHasBeenTriggered()) {
                            event.getEventFunction().run();
                            event.setHasBeenTriggered(true);
                            lastTriggeredEventIndex = i;
                        }
                    }
                }
                    
                if (this.shouldStopAtEnd) {
                    CMConfig.setRobotVelocityMPSandDPS(0, 0, 0);
                }        
            } else {
                double[] currentVelocityComponents = CMConfig.getRobotVelocityMPSandDPS();
                double currentVelocityMag = calculateMagnitude(currentVelocityComponents[0],
                    currentVelocityComponents[1]);
                if(this.lastDesiredVelocity == -1) {
                    this.lastDesiredVelocity = Math.max(currentVelocityMag, this.minStartVelocity);
                }
                    
                if(lastMinPoint == null) {
                    this.lastMinPointPathPoint = path[0];
                    this.lastMinPoint = path[0].getTranslationalPoint();
                    this.currentMinPointPathPoint = path[1];
                    this.currentMinPoint = path[1].getTranslationalPoint();
                    this.nextMinPoint = path[2].getTranslationalPoint();
                    this.endPoint = path[path.length-1];
                }

                if(rotationDeadline == null) {
                    if(this.rotationDeadlines.length == 0) {
                        this.rotationDeadline = new CMRotation(robotRot,
                            0, 1,
                            CMConfig.getDefaultMaxDesiredRotationalVelocity(),
                            CMConfig.getDefaultMaxDesiredRotationalAcceleration(),
                            CMConfig.getDefaultMaxDesiredRotationalDeceleration(),
                            CMConfig.getDefaultAngleCorrectionRange(),
                            CMConfig.getDefaultMaxRotationCorrectionVelocityDegrees(),
                            CMConfig.getDefaultMinRotationVelocityToMove(),
                            CMConfig.getDefaultMaxToleranceDegrees(),
                            CMConfig.getDefaultDecelerationBufferDegrees());
                    } else {
                        this.rotationDeadline = rotationDeadlines[currentRotationDeadlineIndex];
                    }
                    this.desiredRotationDegrees = rotationDeadline.getAngleDegrees();
                    this.maxRotationVelocityDegrees = this.rotationDeadline.getMaxRotationVelocityDegrees();
                    this.desiredRotationalAccelerationDegrees = this.rotationDeadline.getDesiredRotationalAccelerationDegrees();
                    this.desiredRotatioanalDecelerationDegrees = this.rotationDeadline.getDesiredRotationalDecelerationDegrees();
                    this.rotationDirection = this.rotationDeadline.getRotationDirection();
                    this.rotationCorrectionRangeDegrees = this.rotationDeadline.getAngleCorrectionRange();
                    this.maxRotationCorrectionVelocityDegrees = this.rotationDeadline.getMaxRotationCorrectionVelocityDegrees();
                    this.minRotationVelocityToMove = this.rotationDeadline.getMinRotationVelocityToMoveDegrees();
                    this.maxRotationToleranceDegrees = this.rotationDeadline.getMaxToleranceDegrees();
                    this.decelerationBufferDegrees = this.rotationDeadline.getDecelerationBufferDegrees();
                    this.rotationDeadlineCompletePercent = this.rotationDeadline.getCompleteRotationPercent();
                }

                // Trejectory logic
                double desiredVelocityMag = calculateDesiredTranslationalVelocity(currentVelocityMag, robotX, robotY);
                double rotationalVelocity = calculateDesiredRotationalVelocity(Math.abs(currentVelocityComponents[2]), robotRot);
                
                double[] desaturatedVelocities = desaturateVelocities(desiredVelocityMag, Math.abs(rotationalVelocity));
                desiredVelocityMag = desaturatedVelocities[0];
                rotationalVelocity = desaturatedVelocities[1] * Math.signum(rotationalVelocity);

                double travelDistance = ((desiredVelocityMag + currentVelocityMag) / 2) * averageFrameTime;
                // Robot distence along path, distence to lastMinPoint + lastMinPoint distence from start or
                double distanceToNext = calculateMagnitude(nextMinPoint.x - robotX, nextMinPoint.y - robotY);
                double distanceToLast = calculateMagnitude(lastMinPoint.x - robotX, lastMinPoint.y - robotY);
                
                while(distanceToNext - distanceToLast < 0 && this.currentMinPointIndex < path.length-2) {
                    this.currentMinPointIndex++;
                    this.lastMinPointPathPoint = path[this.currentMinPointIndex-1];
                    this.lastMinPoint = lastMinPointPathPoint.getTranslationalPoint();
                    this.currentMinPointPathPoint = path[this.currentMinPointIndex];
                    this.currentMinPoint = currentMinPointPathPoint.getTranslationalPoint();
                    this.nextMinPoint = path[this.currentMinPointIndex+1].getTranslationalPoint();
                    distanceToNext = calculateMagnitude(nextMinPoint.x - robotX, nextMinPoint.y - robotY);
                    distanceToLast = calculateMagnitude(lastMinPoint.x - robotX, lastMinPoint.y - robotY);
                }

                double distanceFromStart = distanceToLast + this.lastMinPointPathPoint.getDistanceFromStart();
                double percentTravel = distanceFromStart / endPoint.getDistanceFromStart();
                    
                if(percentTravel >= this.rotationDeadlineCompletePercent) {
                    for(int i = currentRotationDeadlineIndex+1; i < rotationDeadlines.length; i++) {
                        if(percentTravel >= this.rotationDeadlineCompletePercent) {
                            lastRotationDeadlineIndex = currentRotationDeadlineIndex;
                            currentRotationDeadlineIndex = i;
                            this.rotationDeadline = rotationDeadlines[currentRotationDeadlineIndex];
                            this.desiredRotationDegrees = rotationDeadline.getAngleDegrees();
                            this.maxRotationVelocityDegrees = this.rotationDeadline.getMaxRotationVelocityDegrees();
                            this.desiredRotationalAccelerationDegrees = this.rotationDeadline.getDesiredRotationalAccelerationDegrees();
                            this.desiredRotatioanalDecelerationDegrees = this.rotationDeadline.getDesiredRotationalDecelerationDegrees();
                            this.rotationDirection = this.rotationDeadline.getRotationDirection();
                            this.rotationCorrectionRangeDegrees = this.rotationDeadline.getAngleCorrectionRange();
                            this.maxRotationToleranceDegrees = this.rotationDeadline.getMaxToleranceDegrees();
                            this.decelerationBufferDegrees = this.rotationDeadline.getDecelerationBufferDegrees();
                            this.rotationDeadlineCompletePercent = this.rotationDeadline.getCompleteRotationPercent();
                        } else {
                            break;
                        }
                    }
                }

                for(int i = lastTriggeredEventIndex+1; i < this.events.length-1; i++) {
                    CMEvent event = events[i];
                    double triggerPercent = event.getEventTriggerPercent();
                    if(percentTravel >= triggerPercent && !event.getHasBeenTriggered()) {
                            event.getEventFunction().run();
                            event.setHasBeenTriggered(true);
                            lastTriggeredEventIndex = i;
                    } else {
                        break;
                    }
                }
            
                Logger.recordOutput("CrowMotion/Debug/DesiredRotation", this.rotationDeadline.getAngleDegrees());
                this.goalPointIndex = -1;
                Point2D.Double goalPointRangeEndPose = new Point2D.Double();
                double distanceToGoalPointEndRange = 0;
                for(int i = this.currentMinPointIndex; i < path.length-2; i++) {
                    goalPointRangeEndPose = path[i].getTranslationalPoint();
                    distanceToGoalPointEndRange = calculateMagnitude(goalPointRangeEndPose.x - robotX,
                        goalPointRangeEndPose.y - robotY);
                    if(travelDistance * lookAheadMultiplier < distanceToGoalPointEndRange) {
                        goalPointIndex = i+1;
                        break;
                    }
                }
            
                if(goalPointIndex == -1) {
                    goalPointIndex = path.length-1;
                    goalPointRangeEndPose = endPoint.getTranslationalPoint();
                }

                double[] velocityDir = new double[] {goalPointRangeEndPose.x - robotX, goalPointRangeEndPose.y - robotY};

                double[] velocityComponents = calculateComponentVelocities(velocityDir[0], velocityDir[1], desiredVelocityMag);
                Logger.recordOutput("CrowMotion/Debug/Velocity/MaxTranslationalVelocity", this.maxDesiredTranslationalVelocity);
                Logger.recordOutput("CrowMotion/Debug/Velocity/DesiredTranslationalAcceleration", desiredTranslationalAcceleration);
                Logger.recordOutput("CrowMotion/Debug/Velocity/DesiredVelocityMag", desiredVelocityMag);
                Logger.recordOutput("CrowMotion/Debug/Velocity/CurrentVelocityMag", currentVelocityMag);
                Logger.recordOutput("CrowMotion/Debug/TravelDistance", travelDistance);
                Logger.recordOutput("CrowMotion/Debug/GoalPointRangeEndPoseIndex", goalPointIndex);
                Logger.recordOutput("CrowMotion/Debug/GoalPoint", new double[] {goalPointRangeEndPose.x, goalPointRangeEndPose.y});
                Logger.recordOutput("CrowMotion/Debug/Velocity/EndVelocity", velocityComponents);
                //Logger.recordOutput("CrowMotion/Debug/TrajLength", this.path[this.path.length-1].getDistenceFromStart());
                //Logger.recordOutput("CrowMotion/Debug/PercentCompletion", percentTravel);

                if(this.preventTranslation) {
                    CMConfig.setRobotVelocityMPSandDPS(0, 0, rotationalVelocity);
                } else {
                    CMConfig.setRobotVelocityMPSandDPS(velocityComponents[0], velocityComponents[1], rotationalVelocity);
                }
            }
        } else {
            loadPath(currentTime);
        }
    }
            
    public boolean isCompleted() {
        return this.isComplete;
    }

    private boolean shouldEnd(double robotX, double robotY, double robotRot, long currentTime) {
        // If in x and y tolerance, end those motions but continue rotation tolerance check for inputted time period
        boolean inTranslationalTolerance = Math.abs(robotX - this.endRobotState[0]) < this.positionTolerance[0] &&
            Math.abs(robotY - this.endRobotState[1]) < this.positionTolerance[1];
        boolean inRotTolorence = Math.abs(robotRot - desiredRotationDegrees) < this.maxRotationToleranceDegrees && this.rotationDeadlineCompletePercent == 1;
        
        if(inRotTolorence && this.rotationSettleEndTime == -1) {
            this.rotationSettleEndTime = currentTime + (long)(rotationSettleTime * 1000);
        } else if(!inRotTolorence && this.rotationSettleEndTime != -1) {
            this.rotationSettleEndTime = -1;
        }

        Logger.recordOutput("CrowMotion/Debug/Ending/InToleranceEndTime", this.rotationSettleEndTime - currentTime);
        Logger.recordOutput("CrowMotion/Debug/Ending/InTolerance", inTranslationalTolerance);
        if(inTranslationalTolerance && this.shouldStopAtEnd) {
            this.preventTranslation = true;
        } else if(this.rotationSettleEndTime != -1 &&
            this.rotationSettleEndTime < currentTime) {
            this.preventTranslation = false;
        }
        
        boolean hasTimeElasped = endTime != -1 && currentTime >= endTime;

        boolean doneWithRot = rotationDeadlineCompletePercent != 1 || (this.rotationSettleEndTime != -1 && this.rotationSettleEndTime < currentTime);
        Logger.recordOutput("CrowMotion/Debug/Ending/DoneWithRot",this.rotationSettleEndTime != -1);
        return (inTranslationalTolerance && doneWithRot) || hasTimeElasped;
    }
        
    private double[] desaturateVelocities(double desiredTranslationMag, double desiredRotMag) {
        double velocityNeededForRotation = ((desiredRotMag/360) * this.drivebaseCircumference);
        double requiredModuleVelocity = desiredTranslationMag + velocityNeededForRotation;
            
        if(requiredModuleVelocity > this.maxModuleVelocity) {
            switch (trajectoryPriority) {
                case SPLIT_PROPORTIONALLY:
                    {
                        double proportion = maxModuleVelocity / (desiredTranslationMag + velocityNeededForRotation);
                        double newTranslationalVelocity = desiredTranslationMag * proportion;
                        double newRotationalLinearVelocity = maxModuleVelocity - newTranslationalVelocity;
                        double newRotationalVelocity = (newRotationalLinearVelocity * 360) / drivebaseCircumference;
                        return new double[] {newTranslationalVelocity, newRotationalVelocity};
                    }
                case PREFER_TRANSLATION:
                    {
                        if(desiredTranslationMag > maxModuleVelocity) {
                            return new double[] {desiredTranslationMag, 0};
                        }
                        double newRotationalLinearVelocity = maxModuleVelocity - desiredTranslationMag;
                        double newRotationVelocity = (newRotationalLinearVelocity * 360) / drivebaseCircumference;
                        return new double[] {desiredTranslationMag, newRotationVelocity};
                    }
                case PREFER_ROTATION: 
                    {
                        if(velocityNeededForRotation > maxModuleVelocity) {
                            return new double[] {0, desiredRotMag};
                        }
                        double newTranslationalVelocity = maxModuleVelocity - velocityNeededForRotation;
                        return new double[] {newTranslationalVelocity, desiredRotMag};
                    }
                default:
                    return new double[] {desiredTranslationMag, desiredRotMag};
            }
        }
        return new double[] {desiredTranslationMag, desiredRotMag};
        
    }
    
    private double[] calculateComponentVelocities(double deltaX, double deltaY,
        double translationalVelocityMagnitude) {
        double length = calculateMagnitude(deltaX, deltaY);
        if (length == 0)
            return new double[] { 0, 0 };
        double dx = (translationalVelocityMagnitude * deltaX) / length;
        double dy = (translationalVelocityMagnitude * deltaY) / length;
        return new double[] { dx, dy };
    }
                
    private double calculateDesiredTranslationalVelocity(double currentVelocityMag,
        double robotX, double robotY) {
            
        double distanceToEnd = 0.0;
        if(this.goalPointIndex >= path.length-1) {
            Point2D.Double translationData = this.endPoint.getTranslationalPoint();
            distanceToEnd = calculateMagnitude(translationData.x - robotX, translationData.y - robotY);
        } else {
            double distanceFromRobotToMinPoint = calculateMagnitude(robotX - this.currentMinPoint.x,
            robotY - this.currentMinPoint.y);
            distanceToEnd = (endPoint.getDistanceFromStart() - currentMinPointPathPoint.getDistanceFromStart()) +
            distanceFromRobotToMinPoint;
        }   
            
        if(!shouldDecelerate) {
            double distanceToStartDecelerating = (currentVelocityMag * currentVelocityMag - this.endVelocity * this.endVelocity)
                / (2 * this.desiredTranslationalDecceleration) + this.distanceAtEndVelocity;
            shouldDecelerate = distanceToEnd < distanceToStartDecelerating;
        }
    
        double desiredVelocity = 0;
        if(shouldDecelerate) {
            if(firstDecelerationFrame) {
                this.firstDecelerationFrame = false;
                this.velocityDelta = currentVelocityMag - this.endVelocity;
                this.initialDistanceToEnd = distanceToEnd;
            }
            double percent = ((distanceToEnd - this.distanceAtEndVelocity) / initialDistanceToEnd);
            Logger.recordOutput("CrowMotion/Debug/Percent", percent);
            Logger.recordOutput("CrowMotion/Debug/VelocityDelta", velocityDelta);
            Logger.recordOutput("CrowMotion/Debug/InitialDistenceToEnd", this.initialDistanceToEnd);
            desiredVelocity = Math.max((velocityDelta * clamp(0, 1, percent) + endVelocity), this.endVelocity);
        } else {
            firstDecelerationFrame = true;
            desiredVelocity = lastDesiredVelocity + this.desiredTranslationalAcceleration * this.averageFrameTime;
        }

        Logger.recordOutput("CrowMotion/Debug/DistanceToEnd", distanceToEnd);
        lastDesiredVelocity = Math.min(desiredVelocity, this.maxDesiredTranslationalVelocity);
        return lastDesiredVelocity;
    }
    
    private double calculateDesiredRotationalVelocity(double currentRotationalVelocity, double robotRot) {
        if(this.currentRotationDeadlineIndex != this.lastRotationDeadlineIndex) {
            this.lastDesiredRotationalVelocity = 0;
            this.firstRotationDecelerationFrame = true;
            this.enteredCorrectionRange = false;
            this.shouldDecelerateRotation = false;
            this.initalRotationalVelocity = 0;
            this.initialDegreesToEnd = 0;
        }
        double rawDegreesToGoal = robotRot - this.desiredRotationDegrees;
        rawDegreesToGoal = (rawDegreesToGoal + 540) % 360 - 180;
        double degreesToGoal = Math.abs(rawDegreesToGoal); 
        double degreesToOffsetGoal = degreesToGoal - this.rotationCorrectionRangeDegrees; 

        double dirToGoal = Math.signum(rawDegreesToGoal) * -1;
        if(!this.enteredCorrectionRange) {
            this.enteredCorrectionRange = degreesToGoal < this.rotationCorrectionRangeDegrees;
        }
    
        double realDesiredVelocity = 0;
        if(enteredCorrectionRange) {
            double desiredVelocity = Math.max(maxRotationCorrectionVelocityDegrees * (degreesToGoal / this.rotationCorrectionRangeDegrees), this.minRotationVelocityToMove);
            if(degreesToGoal < this.maxRotationToleranceDegrees) {
                desiredVelocity = 0;
            }
            realDesiredVelocity = desiredVelocity * dirToGoal;
        } else {
            double desiredVelocity = 0;
            if(!this.shouldDecelerateRotation) {
                double distanceToStartDecelerating = (currentRotationalVelocity * currentRotationalVelocity - maxRotationCorrectionVelocityDegrees * maxRotationCorrectionVelocityDegrees)
                    / (2 * this.desiredRotatioanalDecelerationDegrees) + this.decelerationBufferDegrees; 
                this.shouldDecelerateRotation = degreesToOffsetGoal < distanceToStartDecelerating;
            }
    
            if(this.shouldDecelerateRotation) {
                if(this.firstRotationDecelerationFrame) {
                    this.firstRotationDecelerationFrame = false;
                    this.initalRotationalVelocity = currentRotationalVelocity;
                    this.initialDegreesToEnd = degreesToOffsetGoal;
                }
                double percent = degreesToOffsetGoal / initialDegreesToEnd;
                desiredVelocity = Math.max((initalRotationalVelocity * clamp(0, 1, percent) + maxRotationCorrectionVelocityDegrees),0);
            } else {
                this.firstDecelerationFrame = true;
                desiredVelocity = lastDesiredRotationalVelocity + 
                (this.desiredRotationalAccelerationDegrees * averageFrameTime);
            }
            double realRotationDirection = this.rotationDirection;
            if(this.rotationDirection == 0) {
                realRotationDirection = dirToGoal;
            }
            this.lastDesiredRotationalVelocity = desiredVelocity;
            realDesiredVelocity = desiredVelocity * realRotationDirection;   
        }
        
        Logger.recordOutput("CrowMotion/Debug/Rotation/RobotRotation", robotRot);
        Logger.recordOutput("CrowMotion/Debug/Rotation/DegreesToOffsetGoal", degreesToOffsetGoal);
        Logger.recordOutput("CrowMotion/Debug/Rotation/DegreesToGoal", degreesToGoal);
        Logger.recordOutput("CrowMotion/Debug/Rotation/ShouldDecelerateRotation", shouldDecelerateRotation);
        Logger.recordOutput("CrowMotion/Debug/Rotation/EnteredCorrectionRange", enteredCorrectionRange);

        Logger.recordOutput("CrowMotion/Debug/Rotation/DesiredRotationVelocity", realDesiredVelocity);
        Logger.recordOutput("CrowMotion/Debug/Rotation/CurrentRotationVelocity", currentRotationalVelocity);

        Logger.recordOutput("CrowMotion/Debug/Rotation/CurrentRotationDeadlineIndex", currentRotationDeadlineIndex);
        Logger.recordOutput("CrowMotion/Debug/Rotation/LastRotationDeadlineIndex", lastRotationDeadlineIndex);
        
        this.lastRotationDeadlineIndex = this.currentRotationDeadlineIndex;
        double rotationVelocityMag = Math.abs(realDesiredVelocity);
        double rotationDirSing = Math.signum(realDesiredVelocity);
        realDesiredVelocity = Math.min(rotationVelocityMag, maxRotationVelocityDegrees) * rotationDirSing;
        return realDesiredVelocity;
    }

    private double calculateMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    @SuppressWarnings("unused")
    private double calculateMagnitudeRelative(double x, double y) {
        return (x * x + y * y);
    }

    private void loadPath(long currentTime) {
        if (pathGenResult == null && futurePath.isDone()) {
            pathGenResult = futurePath.getNow(new CMPathGenResult(null, null, null));
            path = pathGenResult.path;
            rotationDeadlines = pathGenResult.rotationDeadlines;
            events = pathGenResult.events;
            Logger.recordOutput("CrowMotion/" + pathName, CMPathPoint.point2dToTranslation2D(path));
            this.endTime = currentTime + (long)(this.maxTime * 1000);
        }
    }

    private double clamp(double min, double max, double value) {
        return Math.min(Math.max(value, min), max);
    }

    
}

