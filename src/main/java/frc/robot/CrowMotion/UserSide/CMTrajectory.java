package frc.robot.CrowMotion.UserSide;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.Logger;

import frc.robot.CrowMotion.Library.CMPathGenerator;
import frc.robot.CrowMotion.Library.CMPathPoint;

public class CMTrajectory {

    private String pathName;
    private double maxDesiredTranslationalVelocity;
    private double desiredTranslationalAcceleration;
    private double desiredTranslationalDecceleration;

    private double maxDesiredRotationalVelocity;
    private TrajectoryPriority trajectoryPriority;
    private double endVelocity;
    private double distenceAtEndVelocity;

    private boolean shouldStopAtEnd;
    private double[] positionTolorence;
    private double lookAHeadMult;
    private double maxTime;
    private CompletableFuture<CMPathPoint[]> futurePath;

    private CMPathPoint[] path = null;
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



    private double maxFrameTime;
                    
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
                double maxDesiredRotationalVelocity,
                TrajectoryPriority trajectoryPriority,
                double endVelocity,
                double distenceAtEndVelocity,
                boolean shouldStopAtEnd,
                double[] positionTolorence,
                double lookAHeadMult,
                double maxTime) {
            this.pathName = pathName;
            this.maxDesiredTranslationalVelocity = maxDesiredTranslationalVelocity;
            this.desiredTranslationalAcceleration = desiredTranslationalAcceleration;
            this.desiredTranslationalDecceleration = desiredTranslationalDecceleration;
            this.maxDesiredRotationalVelocity = maxDesiredRotationalVelocity;
            this.trajectoryPriority = trajectoryPriority;
            this.endVelocity = endVelocity;
            this.distenceAtEndVelocity = distenceAtEndVelocity;
            this.shouldStopAtEnd = shouldStopAtEnd;
            this.positionTolorence = positionTolorence;
            this.lookAHeadMult = lookAHeadMult;
            this.maxTime = maxTime;
            assert controlPoints.length >= 1 : "For" + pathName + "CrowMotion paths need at least one control point";
            assert endVelocity > maxDesiredTranslationalVelocity : "For" + pathName + "CrowMotion End Velocities must be = or less then max translational velocity";
    
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
        
            if (frameStartTime != -1) {
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
            this.isComplete = shouldEnd(robotPosition);
            Logger.recordOutput("CrowMotion/" + pathName + "IsComplete", isComplete);
            if (this.isComplete) {
                this.endTime = -1;
                if (this.shouldStopAtEnd) {
                    CMConfig.setRobotVelocityMPSandDPS(0, 0, 0);
                }
            } else {
                loadPath();
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
                    // Trejectory logic
                    double desiredVelocityMag = calculateDesiredVelocity(currentVelocityMag, robotPosition);
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
                }
    
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
                CMConfig.setRobotVelocityMPSandDPS(velocityComponents[0], velocityComponents[1], 0);
                }
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
            return inXTolorence && inYTolorence;// && inRotationTolorence || hasTimeElasped;
        }
    
        private boolean inTolorenceOfPoint(double x1, double y1, double x2, double y2, double tolorence) {
            boolean inXTolorence = Math.abs(x1 - x2) < tolorence;
            boolean inYTolorence = Math.abs(y1 - x2) < tolorence;
            return inXTolorence && inYTolorence;
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
            
        private double calculateDesiredVelocity(double currentVelocityMag,
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
            shouldDecelerate = distenceToEnd  < distenceToStartDecelerating;
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

    private double calculateMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    private double calculateMagnitudeRelative(double x, double y) {
        return (x * x + y * y);
    }

    private void loadPath() {
        if (path == null && futurePath.isDone()) {
            path = futurePath.getNow(new CMPathPoint[] {});
            Logger.recordOutput("CrowMotion/" + pathName, CMPathPoint.point2dToTranslation2D(path));
            this.endTime = System.currentTimeMillis() + (long)(this.maxTime * 1000);
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

