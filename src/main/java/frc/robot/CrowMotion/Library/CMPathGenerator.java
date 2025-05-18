package frc.robot.CrowMotion.Library;

import frc.robot.CrowMotion.CMAutonPoint;
import frc.robot.CrowMotion.CMEvent;
import frc.robot.CrowMotion.CMRotation;
import frc.robot.CrowMotion.CrowMotionConfig;
import frc.robot.CrowMotion.CMRotation.RotationDirrection;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CMPathGenerator {

    public static PathPoint[] generateCMPath(double maxVelocity, CMAutonPoint[] controlPoints, CMRotation[] rotations, CMEvent[] events) {
            Point2D[] translationData;
            double assumedFrameTime = .015;
            double pointsPerMeter = (1 / (maxVelocity * assumedFrameTime)) * 2;
            if(controlPoints.length == 1) {
                double[] robotPosition = CrowMotionConfig.getRobotPositionMetersAndDegrees();
                controlPoints = new CMAutonPoint[] {new CMAutonPoint(robotPosition[0], robotPosition[1]), controlPoints[0]};
            }
            if(controlPoints.length == 1 || controlPoints.length == 2) {
                translationData = generateLinearPointArray(controlPoints, pointsPerMeter);
            } else {
                translationData = null; // TODO: Add Bezier curve alg
            }

            double currentRotation = CrowMotionConfig.getRobotPositionMetersAndDegrees()[2];
            double[][] detlaPerPoint = new double[rotations.length][2];
            for(int i = 0; i < rotations.length; i++) {
                int pointsForRotation = (int)(rotations[i].getCompleteRotationPercent() * translationData.length);
                double startPoint =  i == 0 ? currentRotation : rotations[i-1].angleDegrees();
                double deltaForRotation = 0;
                if(rotations[i].getRotationDirrection() == RotationDirrection.POSITIVE) {
                    double delta = startPoint - rotations[i].angleDegrees();
                    deltaForRotation = (360 - delta);
                } else {
                    double delta = startPoint - rotations[i].angleDegrees();
                    deltaForRotation = delta * Math.signum(-1);
                }
                detlaPerPoint[i][0] = deltaForRotation / (double)pointsForRotation;
                detlaPerPoint[i][1] = rotations[i].angleDegrees();
            }
            
            PathPoint[] path = new PathPoint[translationData.length];
            double lastRotation = currentRotation;
            int currentRotationCheckPoint = 0;
            for(int i = 0; i < translationData.length; i++) {
                path[i] = new PathPoint(translationData[0], lastRotation, null);
                if(detlaPerPoint[currentRotationCheckPoint][1] == lastRotation) {
                    if(detlaPerPoint.length-1 >= currentRotationCheckPoint) {
                        detlaPerPoint[0][0] = 0;
                    } else {
                        currentRotationCheckPoint++;
                    }
                }
                lastRotation += detlaPerPoint[currentRotationCheckPoint][0];
            }
            
            for(int i = 0; i < events.length; i++) {
                int triggerIndex = (int)(events[i].getEventTriggerPercent() * translationData.length);
                path[triggerIndex].setEvent(events[i]);
            }
            
        return path;
    }

    private static Point2D[] generateLinearPointArray(CMAutonPoint[] controlPoints, double pointsPerMeter) {
        double xDelta = controlPoints[1].getX() - controlPoints[0].getX();
        double yDelta = controlPoints[1].getY() - controlPoints[0].getY();
        
        double distance = Math.sqrt(xDelta * xDelta + yDelta * yDelta);
        
        int numberOfPoints = (int) (distance * pointsPerMeter) + 1;
        Logger.recordOutput("CrowMotion/PointsPerMeter", numberOfPoints);
        Point2D[] points = new Point2D[numberOfPoints];
        
        for (int i = 0; i < numberOfPoints; i++) {
            double t = (double) i / (numberOfPoints - 1);
            double x = controlPoints[0].getX() + t * xDelta;
            double y = controlPoints[0].getY() + t * yDelta;
            points[i] = new Point2D.Double(x, y);
        }
        
        return points;
    }

    public static void writePointsToCSV(Point2D[] points, String filename) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
            writer.println("x,y");
            for (Point2D point : points) {
                writer.printf("%.4f,%.4f%n", point.getX(), point.getY());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    //writePointsToCSV(points, "logs/log.csv");

}
