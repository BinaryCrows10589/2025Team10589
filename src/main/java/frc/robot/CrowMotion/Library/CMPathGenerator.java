package frc.robot.CrowMotion.Library;

import java.awt.geom.Point2D;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.GenericConstants.RobotModeConstants;
import frc.robot.CrowMotion.UserSide.CMAutonPoint;
import frc.robot.CrowMotion.UserSide.CMConfig;
import frc.robot.CrowMotion.UserSide.CMEvent;
import frc.robot.CrowMotion.UserSide.CMRotation;

public class CMPathGenerator {

    public static CompletableFuture<CMPathGenResult> generateCMPathAsync(String pathTime,
            CMAutonPoint[] controlPoints, double initialRotation, CMRotation[] rotations, CMEvent[] events,
            double pointsPerMeter) {
        CompletableFuture<CMPathGenResult> future = new CompletableFuture<>();

        Notifier[] holder = new Notifier[1]; // to close from outside the Notifier thread
        RobotModeConstants.startPathGenTime = System.currentTimeMillis();
        holder[0] = new Notifier(() -> {
            try {
                CMPathGenResult result = createCMPath(pathTime, controlPoints, initialRotation, rotations, events,
                        pointsPerMeter);
                future.complete(result);
            } catch (Exception e) {
                future.completeExceptionally(e);
            } finally {
                // Close Notifier safely from a new thread to avoid closing from itself
                new Thread(() -> {
                    holder[0].close();
                }).start();
            }
        });

        holder[0].startSingle(0);

        return future;
    }

    private static CMPathGenResult createCMPath(String pathName, CMAutonPoint[] controlPoints,
            double initialRotation, CMRotation[] rotations, CMEvent[] events, double pointsPerMeter) {
        Point2D.Double[] translationData;
        double[] robotPosition = CMConfig.getRobotPositionMetersAndDegrees();

        if (controlPoints.length == 1) {
            controlPoints = new CMAutonPoint[] { new CMAutonPoint(robotPosition[0], robotPosition[1]),
                    controlPoints[0] };
        }
        if (controlPoints.length <= 2) {
            translationData = generateLinearPointArray(controlPoints, pointsPerMeter);
        } else {
            translationData = generateBezierPointArray(controlPoints, pointsPerMeter);
        }
        
        CMPathPoint[] path = new CMPathPoint[translationData.length];
        ArrayList<Translation2d> translationsForLogging = new ArrayList<Translation2d>();
        
        Point2D.Double translationLast = null;
        double currentDistenceFromStart = 0.0;
        for (int i = 0; i < translationData.length; i++) {
            Point2D.Double translation = translationData[i];
            double distenceFromLast = 0;
            if(translationLast != null) {
                distenceFromLast = translation.distance(translationLast);
            }
            currentDistenceFromStart += distenceFromLast;
            translationLast = translation;
            path[i] = new CMPathPoint(translation, currentDistenceFromStart);
            
            if (i % 10 == 0) {
                translationsForLogging.add(new Translation2d(translation.x, translation.y));
            }
        }

        return new CMPathGenResult(path, rotations, events);
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
        int numberOfPoints = (int) (estimatedLength * pointsPerMeter * 1.3) + 1;

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

}
