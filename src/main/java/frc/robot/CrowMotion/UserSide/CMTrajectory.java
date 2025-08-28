package frc.robot.CrowMotion.UserSide;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.Logger;

import frc.robot.CrowMotion.Library.CMPathGenResult;
import frc.robot.CrowMotion.Library.CMPathGenerator;
import frc.robot.CrowMotion.Library.CMPathPoint;

public class CMTrajectory {

    private String pathName;
    private double maxDesiredTranslationalVelocity;
    private double desiredTranslationalAcceleration;
    private double desiredTranslationalDecceleration;
    
    private double rotationSettleTime;
    private long rotationSettleEndTime = -1;
    private boolean preventTranslation = false;

    private TrajectoryPriority trajectoryPriority;
    private double endVelocity;
    private double distenceAtEndVelocity;

    private boolean shouldStopAtEnd;
    private double[] positionTolorence;
    private double lookAHeadMult;
    private double maxTime;
    private CompletableFuture<CMPathGenResult> futurePath;

    private CMPathGenResult pathGenResult = null;
    private CMPathPoint[] path = null;
    private int[] rotationDeadlines = null;
    private double[] endRobotState;
    private long endTime = -1;
    private boolean isComplete = false;

    private long frameStartTime = -1;
    private double averageFrameTime = .02;

    private int currentMinPointIndex = 1;
    private Point2D.Double lastMinPoint = null;
    private Point2D.Double nextMinPoint = null;

    private Point2D.Double currentMinPoint = null;
    private CMPathPoint currentMinPointPathPoint = null;
    private CMPathPoint endPoint = null;
    private int goalPointIndex;

    private boolean shouldDecelerate = false;
    private double velocityDelta = 0;
    private boolean firstDecelerationFrame = true;
    private double initialDistenceToEnd = 0;

    private int lastRotationDeadlineIndex = -1;
    private int currentRotationDeadlineIndex = 1;
    private CMRotation rotationDeadline = null;
    private double desiredRotationDegrees;
    private double maxRotationVelocityDegrees;
    private double desiredRotationalAccelerationDegrees;
    private double desiredRotatioanalDecelerationDegrees;
    private int rotationDirection = 0;
    private double rotationCorrectionRangeDegrees = 0;
    private double maxRotationCorrectionVelocityDegrees = 0;
    private double minRotationVelocityToMove = 0;
    private double maxRotationTolorenceDegrees;
    private double decelerationBufferDegrees;

    private boolean firstRotationDecelerationFrame = true;
    private boolean enteredCorrectionRange = false;
    private boolean shouldDecelerateRotation = false;
    private double initalRotationalVelocity = 0;
    private double initialDegreesToEnd = 0;

    private double maxFrameTime;

    private double drivebaseCircumference;
    private double maxModuleVelocity;

    //TODO: After getting robot pose as an array instentiat 3 vars and pass those around
    public static enum TrajectoryPriority {
        PREFER_ROTATION,
        PREFER_TRANSLATION,
        SPLIT_PROPORTIONALLY
    }

    public CMTrajectory(String pathName, CMAutonPoint[] controlPoints, double initialRotation,
        CMRotation[] rotations, CMEvent[] events,
        double pointsPerMeter,
        double maxDesiredTranslationalVelocity,
        double desiredTranslationalAcceleration,
        double desiredTranslationalDecceleration,
        TrajectoryPriority trajectoryPriority,
        double endVelocity,
        double distenceAtEndVelocity,
        boolean shouldStopAtEnd,
        double[] positionTolorence,
        double lookAHeadMult,
        double rotationSettleTime, double maxTime) {
            
        this.pathName = pathName;
        this.maxDesiredTranslationalVelocity = maxDesiredTranslationalVelocity;
        this.desiredTranslationalAcceleration = desiredTranslationalAcceleration;
        this.desiredTranslationalDecceleration = desiredTranslationalDecceleration;
        this.trajectoryPriority = trajectoryPriority;
        this.endVelocity = endVelocity;
        this.distenceAtEndVelocity = distenceAtEndVelocity;
        this.shouldStopAtEnd = shouldStopAtEnd;
        this.positionTolorence = positionTolorence;
        this.lookAHeadMult = lookAHeadMult;
        this.maxTime = maxTime;
        this.rotationSettleTime = rotationSettleTime;

        assert controlPoints.length >= 1 : "For" + pathName + " CrowMotion paths need at least one control point";
        assert this.endVelocity > maxDesiredTranslationalVelocity : "For" + pathName + " CrowMotion End Velocities must be = or less then max translational velocity";
        assert this.maxTime > 0 : "For" + pathName + " Max Time must be greater than 0";
        assert this.lookAHeadMult > 0 : "For" + pathName + " Look a head mult must be greater than 0";
        assert this.distenceAtEndVelocity > 0 : "For" + pathName + " Distence at end velocity must be greater than 0";
        assert this.rotationSettleTime > 0 : "For" + pathName + " Time at final position must be greater than 0";

        this.drivebaseCircumference = CMConfig.getDrivebaseCircumference();
        this.maxModuleVelocity = CMConfig.getRobotProfile().getMaxPossibleAverageSwerveModuleMPS();

        // TODO: Add all invalid path asserts
        this.futurePath = CMPathGenerator.generateCMPathAsync("TestBezier",
                controlPoints, initialRotation, rotations, events, pointsPerMeter);
        CMAutonPoint lastPoint = controlPoints[controlPoints.length - 1];
        double endRotation = rotations.length == 0 ? initialRotation
                : rotations[rotations.length - 1].getAngleDegrees();
        this.endRobotState = new double[] { lastPoint.getX(), lastPoint.getY(), endRotation };
    }
    // I want to clamp the vector to have a max change per frame, that is equal to the max accelration, but insteed of magnatude also takes into acount rotations. It shoud rotate the vector over just flipping dir 
    /* 3. When doing 2 also approximate the desired rotation by the end of this frame
        * 4. Desaturate the translation and rotaiton velocity in acordence with the path priority given
        * 5. Calculate translational vector
        * 6. Apply velocities
        */
    public void runTrejectoryFrame() {
        long currentTime = System.currentTimeMillis();
    
        if(frameStartTime != -1) {
            double frameTime = (currentTime - frameStartTime) / 1000.0; // Convert to seconds
            if(frameTime > maxFrameTime) {
                maxFrameTime = frameTime;
                Logger.recordOutput("CrowMotion/Debug/MaxFrameTime", maxFrameTime);
            }
            averageFrameTime = (averageFrameTime * 0.9) + (frameTime * 0.1); // EWMA smoothing
            Logger.recordOutput("CrowMotion/Debug/FrameTime", averageFrameTime);
        }
        frameStartTime = currentTime;

        double[] robotPosition = CMConfig.getRobotPositionMetersAndDegrees();
        this.isComplete = shouldEnd(robotPosition, currentTime);
        Logger.recordOutput("CrowMotion/" + pathName + "IsComplete", isComplete);
        if(this.isComplete) {
            this.endTime = -1;
            this.rotationSettleEndTime = -1;
            this.preventTranslation = false;
            this.lastRotationDeadlineIndex = -1;
            this.currentRotationDeadlineIndex = 1;
            this.lastMinPoint = null;
            this.currentMinPointPathPoint = null;
            this.currentMinPoint = null;
            this.nextMinPoint = null;
            this.endPoint = null;
            //TODO: Reset all varialbes needed to allow for reuse.
            if (this.shouldStopAtEnd) {
                CMConfig.setRobotVelocityMPSandDPS(0, 0, 0);
            }
        } else {
            loadPath(currentTime);
            if (this.path != null) {
                double[] currentVelocityComponents = CMConfig.getRobotVelocityMPSandDPS();
                double currentVelocityMag = calculateMagnitude(currentVelocityComponents[0],
                    currentVelocityComponents[1]);
                if(lastMinPoint == null) {
                    this.lastMinPoint = path[0].getTranslationalPoint();
                    this.currentMinPointPathPoint = path[1];
                    this.currentMinPoint = path[1].getTranslationalPoint();
                    this.nextMinPoint = path[2].getTranslationalPoint();
                    this.endPoint = path[path.length-1];
                }

                if(rotationDeadline == null) {
                    if(this.rotationDeadlines.length == 0 ) {
                        this.rotationDeadline = new CMRotation(robotPosition[2],
                            0, 1,
                            CMConfig.getDefaultMaxDesiredRotationalVelocity(),
                            CMConfig.getDefaultMaxDesiredRotationalAcceleration(),
                            CMConfig.getDefaultMaxDesiredRotationalDeceleration(),
                            CMConfig.getDefaultAngleCorrectionRange(),
                            CMConfig.getDefaultMaxRotationCorrectionVelocityDegrees(),
                            CMConfig.getDefaultMinRotationVelocityToMove(),
                            CMConfig.getDefaultMaxTolorenceDegrees(),
                            CMConfig.getDefaultDecelerationBufferDegrees());
                    } else {
                        this.rotationDeadline = path[rotationDeadlines[0]].getDesiredRotation();
                    }
                    this.desiredRotationDegrees = rotationDeadline.getAngleDegrees();
                    this.maxRotationVelocityDegrees = this.rotationDeadline.getMaxRotationVelocityDegrees();
                    this.desiredRotationalAccelerationDegrees = this.rotationDeadline.getDesiredRotationalAccelerationDegrees();
                    this.desiredRotatioanalDecelerationDegrees = this.rotationDeadline.getDesiredRotationalDecelerationDegrees();
                    this.rotationDirection = this.rotationDeadline.getRotationDirrection();
                    this.rotationCorrectionRangeDegrees = this.rotationDeadline.getAngleCorrectionRange();
                    this.maxRotationCorrectionVelocityDegrees = this.rotationDeadline.getMaxRotationCorrectionVelocityDegrees();
                    this.minRotationVelocityToMove = this.rotationDeadline.getMinRotationVelocityToMoveDegrees();
                    this.maxRotationTolorenceDegrees = this.rotationDeadline.getMaxTolorenceDegrees();
                    this.decelerationBufferDegrees = this.rotationDeadline.getDecelerationBufferDegrees();
                }
                

                // Trejectory logic
                double desiredVelocityMag = calculateDesiredTranslationalVelocity(currentVelocityMag, robotPosition);
                double rotationalVelocity = calculateDesiredRotationalVelocity(Math.abs(currentVelocityComponents[2]), robotPosition);
                
                double[] desaturatedVelocities = desaturateVelocities(desiredVelocityMag, Math.abs(rotationalVelocity));
                desiredVelocityMag = desaturatedVelocities[0];
                rotationalVelocity = desaturatedVelocities[1] * Math.signum(rotationalVelocity);

                double travelDistence = ((desiredVelocityMag + currentVelocityMag) / 2) * averageFrameTime;
                
                double disToNext = calculateMagnitude(nextMinPoint.x - robotPosition[0], nextMinPoint.y - robotPosition[1]);
                double disToLast = calculateMagnitude(lastMinPoint.x - robotPosition[0], lastMinPoint.y - robotPosition[1]);
                while(disToNext - disToLast < .2 && this.currentMinPointIndex < path.length-2) {
                    this.currentMinPointIndex++;
                    lastMinPoint = path[this.currentMinPointIndex-1].getTranslationalPoint();
                    currentMinPointPathPoint = path[this.currentMinPointIndex];
                    currentMinPoint = currentMinPointPathPoint.getTranslationalPoint();
                    nextMinPoint = path[this.currentMinPointIndex+1].getTranslationalPoint();
                    disToNext = calculateMagnitude(nextMinPoint.x - robotPosition[0], nextMinPoint.y - robotPosition[1]);
                    disToLast = calculateMagnitude(lastMinPoint.x - robotPosition[0], lastMinPoint.y - robotPosition[1]);
                
                    if(rotationDeadlines.length > this.currentRotationDeadlineIndex && rotationDeadlines[this.currentRotationDeadlineIndex-1] < currentMinPointIndex) {
                        this.rotationDeadline = path[rotationDeadlines[this.currentRotationDeadlineIndex]].getDesiredRotation();
                        this.currentRotationDeadlineIndex++;
                        this.desiredRotationDegrees = rotationDeadline.getAngleDegrees();
                        this.maxRotationVelocityDegrees = this.rotationDeadline.getMaxRotationVelocityDegrees();
                        this.desiredRotationalAccelerationDegrees = this.rotationDeadline.getDesiredRotationalAccelerationDegrees();
                        this.desiredRotatioanalDecelerationDegrees = this.rotationDeadline.getDesiredRotationalDecelerationDegrees();
                        this.rotationDirection = this.rotationDeadline.getRotationDirrection();
                        this.rotationCorrectionRangeDegrees = this.rotationDeadline.getAngleCorrectionRange();
                        this.maxRotationTolorenceDegrees = this.rotationDeadline.getMaxTolorenceDegrees();
                        this.decelerationBufferDegrees = this.rotationDeadline.getDecelerationBufferDegrees();
                    }
                }
                Logger.recordOutput("CrowMotion/Debug/DesiredRotation", this.rotationDeadline.getAngleDegrees());
                this.goalPointIndex = -1;
                Point2D.Double goalPointRangeEndPose = new Point2D.Double();
                double disToGoalPointEndRange = 0;
                for(int i = this.currentMinPointIndex; i < path.length-2; i++) {
                    goalPointRangeEndPose = path[i].getTranslationalPoint();
                    disToGoalPointEndRange = calculateMagnitude(goalPointRangeEndPose.x - robotPosition[0],
                        goalPointRangeEndPose.y - robotPosition[1]);
                    if(travelDistence * lookAHeadMult < disToGoalPointEndRange) {
                        goalPointIndex = i+1;
                        break;
                    }
                }
            
                if(goalPointIndex == -1) {
                    goalPointIndex = path.length-1;
                    goalPointRangeEndPose = endPoint.getTranslationalPoint();
                    // Figgure out what to do with velocity in this case. Either slow down to make it or clamp down to end velocity
                    // Maybe do somthing where if stop at end of path is true that modify velocity acordingly if not 
                }

                double percentOfDis = 0;
                if(goalPointIndex >= 1) {
                    percentOfDis = travelDistence / disToGoalPointEndRange;
                }

                double[] velocityDir = new double[] {goalPointRangeEndPose.x - robotPosition[0], goalPointRangeEndPose.y - robotPosition[1]};

                double[] velocityComponents = calculateComponentVelocities(velocityDir[0], velocityDir[1], desiredVelocityMag);
                Logger.recordOutput("CrowMotion/Debug/Velocity/MaxTranslationalVelocity", this.maxDesiredTranslationalVelocity);
                Logger.recordOutput("CrowMotion/Debug/Velocity/DesiredTranslationalAcceleration", desiredTranslationalAcceleration);
                Logger.recordOutput("CrowMotion/Debug/Velocity/DesiredVelocityMag", desiredVelocityMag);
                Logger.recordOutput("CrowMotion/Debug/Velocity/CurrentVelocityMag", currentVelocityMag);
                Logger.recordOutput("CrowMotion/Debug/TravelDis", travelDistence);
                Logger.recordOutput("CrowMotion/Debug/GoalPointRangeEndPoseIndex", goalPointIndex);
                Logger.recordOutput("CrowMotion/Debug/GoalPoint", new double[] {goalPointRangeEndPose.x, goalPointRangeEndPose.y});
                Logger.recordOutput("CrowMotion/Debug/Velocity/EndVelocity", velocityComponents);

                if(this.preventTranslation) {
                    CMConfig.setRobotVelocityMPSandDPS(0, 0, rotationalVelocity);
                } else {
                    CMConfig.setRobotVelocityMPSandDPS(velocityComponents[0], velocityComponents[1], rotationalVelocity);
                }
            }
        }
    }
        
    public boolean isCompleted() {
        return this.isComplete;
    }

    private boolean shouldEnd(double[] robotPosition, long currentTime) {
        // If in x and y tol end those motion but continue rot tolorence check for inputed time period
        boolean inXTolorence = Math.abs(robotPosition[0] - this.endRobotState[0]) < this.positionTolorence[0];
        boolean inYTolorence = Math.abs(robotPosition[1] - this.endRobotState[1]) < this.positionTolorence[1];
        boolean inTranslationalTolorence = inXTolorence && inYTolorence;
        boolean inRotTolorence = Math.abs(robotPosition[2] - desiredRotationDegrees) < this.maxRotationTolorenceDegrees && this.rotationDeadline.getCompleteRotationPercent() == 1;
        if(inRotTolorence && this.rotationSettleEndTime == -1) {
            this.rotationSettleEndTime = currentTime + (int)(rotationSettleTime * 1000);
        } else if(!inRotTolorence) {
            this.rotationSettleEndTime = -1;
        }
        Logger.recordOutput("CrowMotion/Debug/Ending/InTolorenceEndTime", this.rotationSettleEndTime - currentTime);
        Logger.recordOutput("CrowMotion/Debug/Ending/InTolorence", inXTolorence && inYTolorence);
        if(inTranslationalTolorence && this.shouldStopAtEnd) {
            this.preventTranslation = true;
        } else if(this.rotationSettleEndTime != -1 &&
            this.rotationSettleEndTime < currentTime) {
            this.preventTranslation = false;
        }
      
        boolean hasTimeElasped = endTime != -1 && currentTime >= endTime;
        return (inTranslationalTolorence && (this.rotationSettleEndTime != -1 && this.rotationSettleEndTime < currentTime)) || hasTimeElasped;
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
        double[] robotPosition) {
        
        double distenceToEnd = 0.0;
        if(this.goalPointIndex >= path.length-1) {
            Point2D.Double translationData = this.endPoint.getTranslationalPoint();
            distenceToEnd = calculateMagnitude(translationData.x - robotPosition[0], translationData.y - robotPosition[1]);
        } else {
            double distenceFromRobotToMinPoint = calculateMagnitude(robotPosition[0] - this.currentMinPoint.x,
            robotPosition[1] - this.currentMinPoint.y);
            distenceToEnd = (endPoint.getDistenceFromStart() - currentMinPointPathPoint.getDistenceFromStart()) +
            distenceFromRobotToMinPoint;
        }   
        
        if(!shouldDecelerate) {
            double distenceToStartDecelerating = (currentVelocityMag * currentVelocityMag - this.endVelocity * this.endVelocity)
                / (2 * this.desiredTranslationalDecceleration) + this.distenceAtEndVelocity;
            shouldDecelerate = distenceToEnd < distenceToStartDecelerating;
        }

        double desiredVelocity = 0;
        if(shouldDecelerate) {
            if(firstDecelerationFrame) {
                this.firstDecelerationFrame = false;
                this.velocityDelta = currentVelocityMag - this.endVelocity;
                this.initialDistenceToEnd = distenceToEnd;
            }
        double percent = ((distenceToEnd - this.distenceAtEndVelocity) / initialDistenceToEnd);
        Logger.recordOutput("CrowMotion/Debug/Percent", percent);
        Logger.recordOutput("CrowMotion/Debug/VelocityDelta", velocityDelta);
        Logger.recordOutput("CrowMotion/Debug/InitialDistenceToEnd", this.initialDistenceToEnd);
        desiredVelocity = Math.max((velocityDelta * clamp(0, 1, percent) + endVelocity), this.endVelocity);
        } else {
            firstDecelerationFrame = true;
            desiredVelocity = currentVelocityMag + this.desiredTranslationalAcceleration * this.averageFrameTime;
        }

        Logger.recordOutput("CrowMotion/Debug/DistenceToEnd", distenceToEnd);
        
        return Math.min(desiredVelocity, this.maxDesiredTranslationalVelocity);
    }

    private double calculateDesiredRotationalVelocity(double currentRotationalVelocity, double[] robotPosition) {
        if(this.currentRotationDeadlineIndex != this.lastRotationDeadlineIndex) {
            this.firstRotationDecelerationFrame = true;
            this.enteredCorrectionRange = false;
            this.shouldDecelerateRotation = false;
            this.initalRotationalVelocity = 0;
            this.initialDegreesToEnd = 0;
        }
        double rawDegreesToGoal = robotPosition[2] - this.desiredRotationDegrees;
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
            if(degreesToGoal < this.maxRotationTolorenceDegrees) {
                desiredVelocity = 0;
            }
            realDesiredVelocity = desiredVelocity * dirToGoal;
        } else {
            double desiredVelocity = 0;
            if(!this.shouldDecelerateRotation) {
                double distenceToStartDecelerating = (currentRotationalVelocity * currentRotationalVelocity - maxRotationCorrectionVelocityDegrees * maxRotationCorrectionVelocityDegrees)
                    / (2 * this.desiredRotatioanalDecelerationDegrees) + this.decelerationBufferDegrees; 
                this.shouldDecelerateRotation = degreesToOffsetGoal < distenceToStartDecelerating;
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
                desiredVelocity = currentRotationalVelocity + 
                    (this.desiredRotationalAccelerationDegrees * averageFrameTime);
            }
            // Add auto detection of shortest dir
            double realRotationDirection = this.rotationDirection;
            if(this.rotationDirection == 0) {
                realRotationDirection = dirToGoal;
            }
            realDesiredVelocity = desiredVelocity * realRotationDirection;   
        }
        
        Logger.recordOutput("CrowMotion/Debug/Rotation/RobotRotation", robotPosition[2]);
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

    private double calculateMagnitudeRelative(double x, double y) {
        return (x * x + y * y);
    }

    private void loadPath(long currentTime) {
        if (pathGenResult == null && futurePath.isDone()) {
            pathGenResult = futurePath.getNow(new CMPathGenResult(null, null));
            path = pathGenResult.path;
            rotationDeadlines = pathGenResult.rotationDeadlines;
            Logger.recordOutput("CrowMotion/" + pathName, CMPathPoint.point2dToTranslation2D(path));
            this.endTime = currentTime + (long)(this.maxTime * 1000);
        }
    }

    private double clamp(double min, double max, double value) {
        if(value > max) {
            return max;
        } else if(value < min) {
            return min;
        }
        return value;
    }

    
}

