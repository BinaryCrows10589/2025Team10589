package frc.robot.CrowMotion.Library;

import frc.robot.CrowMotion.CMAutonPoint;
import frc.robot.CrowMotion.CMEvent;
import frc.robot.CrowMotion.CMRotation;
import frc.robot.CrowMotion.CrowMotionConfig;
import frc.robot.CrowMotion.CMRotation.RotationDirrection;
import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Notifier;

public class CMPathGenerator {
    private static PathPoint[] createCMPath(double maxVelocity, CMAutonPoint[] controlPoints, CMRotation[] rotations, CMEvent[] events) {
            Point2D.Double[] translationData;
            double assumedFrameTime = .015;
            if(maxVelocity < 4.4) {
                maxVelocity = 4.4;
            }
            double pointsPerMeter = (1 / (maxVelocity * assumedFrameTime)) * 2;
            double[] robotPosition = CrowMotionConfig.getRobotPositionMetersAndDegrees();

            if(controlPoints.length == 1) {
                controlPoints = new CMAutonPoint[] {new CMAutonPoint(robotPosition[0], robotPosition[1]), controlPoints[0]};
            }
            if(controlPoints.length == 1 || controlPoints.length == 2) {
                translationData = generateLinearPointArray(controlPoints, pointsPerMeter);
            } else {
                translationData = generateBezierPointArray(controlPoints, pointsPerMeter); 
            }

            double currentRotation = robotPosition[2];
            int usedPoints = 0;
            double[][] detlaPerPoint = new double[rotations.length][2];
            for(int i = 0; i < rotations.length; i++) {
                int pointsForRotation = (int)(rotations[i].getCompleteRotationPercent() * translationData.length) - usedPoints;
                usedPoints += pointsForRotation;
                double startPoint =  i == 0 ? currentRotation : rotations[i-1].angleDegrees();
                double deltaForRotation = 0;
                if(rotations[i].getRotationDirrection() == RotationDirrection.POSITIVE) {
                    deltaForRotation = (360 - (startPoint - rotations[i].angleDegrees())) % 360;
                } else {
                    deltaForRotation = -(360 + (startPoint - rotations[i].angleDegrees())) % 360;
                }
                if(i == 0) {
                    pointsForRotation -= 1;
                }
                detlaPerPoint[i][0] = deltaForRotation / (double)(pointsForRotation);
                detlaPerPoint[i][1] = rotations[i].angleDegrees();
            }
          

            PathPoint[] path = new PathPoint[translationData.length];
            double lastRotation = currentRotation;
            int currentRotationCheckPoint = 0;
            for(int i = 0; i < translationData.length; i++) {
                path[i] = new PathPoint(translationData[i], lastRotation, null);
                if(Math.abs(lastRotation - detlaPerPoint[currentRotationCheckPoint][1]) < .0000001) {
                    if(detlaPerPoint.length-1 <= currentRotationCheckPoint) {
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

    private static Point2D.Double[] generateLinearPointArray(CMAutonPoint[] controlPoints, double pointsPerMeter) {
        double xDelta = controlPoints[1].getX() - controlPoints[0].getX();
        double yDelta = controlPoints[1].getY() - controlPoints[0].getY();
        
        double distance = Math.sqrt(xDelta * xDelta + yDelta * yDelta);
        
        int numberOfPoints = (int) (distance * pointsPerMeter) + 1;
        Logger.recordOutput("CrowMotion/NumberOfPoints", numberOfPoints);
        Point2D.Double[] points = new Point2D.Double[numberOfPoints];
        
        for (int i = 0; i < numberOfPoints; i++) {
            double t = (double) i / (numberOfPoints - 1);
            double x = controlPoints[0].getX() + t * xDelta;
            double y = controlPoints[0].getY() + t * yDelta;
            points[i] = new Point2D.Double(x, y);
        }
        
        return points;
    }

    public static Point2D.Double[] generateBezierPointArray(CMAutonPoint[] controlPoints, double pointsPerMeter) {
        double estimatedLength = 0;
        for (int i = 0; i < controlPoints.length - 1; i++) {
            double dx = controlPoints[i + 1].getX() - controlPoints[i].getX();
            double dy = controlPoints[i + 1].getY() - controlPoints[i].getY();
            estimatedLength += Math.sqrt(dx * dx + dy * dy);
        }    
        int numberOfPoints = (int)(estimatedLength * pointsPerMeter * 1.3) + 1;

        int degree = controlPoints.length;
        double tStep = 1.0 / (numberOfPoints - 1);
    
        Point2D.Double[] curvePoints = new Point2D.Double[numberOfPoints];
    
        double[] x = new double[degree];
        double[] y = new double[degree];
    
        for (int i = 0; i < degree; i++) {
            x[i] = controlPoints[i].getX();
            y[i] = controlPoints[i].getY();
        }
    
        double[] tempX = new double[degree];
        double[] tempY = new double[degree];
    
        for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++) {
            double t = pointIdx * tStep;
    
            System.arraycopy(x, 0, tempX, 0, degree);
            System.arraycopy(y, 0, tempY, 0, degree);
    
            int len = degree;
            while (len > 1) {
                for (int i = 0; i < len - 1; i++) {
                    tempX[i] = (1 - t) * tempX[i] + t * tempX[i + 1];
                    tempY[i] = (1 - t) * tempY[i] + t * tempY[i + 1];
                }
                len--;
            }
    
            curvePoints[pointIdx] = new Point2D.Double(tempX[0], tempY[0]);
        }
    
        return curvePoints;
    }
    

    public static void writePointsToCSV(PathPoint[] points, String filename) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
            writer.println("x,y");
            for (PathPoint point : points) {
                writer.printf("%.4f,%.4f%n", point.getTranslationalPoint().getX(), point.getTranslationalPoint().getY());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
