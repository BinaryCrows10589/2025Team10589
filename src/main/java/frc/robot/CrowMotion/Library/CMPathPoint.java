package frc.robot.CrowMotion.Library;

import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CrowMotion.UserSide.CMEvent; 
public class CMPathPoint {

    private Point2D.Double translationalPoint;
    private double desiredRotation;
    private CMEvent event;

    /**
     * Constructs an instance with the specified translational point, rotation, and event.
     *
     * @param translationalPoint The x and y coordnets of the path at this point
     * @param desiredRotation The desired rotation angle at this point
     * @param event The event to triggure at this point
     */
    public CMPathPoint(Point2D.Double translationalPoint, double desiredRotation, CMEvent event) {
        this.translationalPoint = translationalPoint;
        this.desiredRotation = wrapAngle(desiredRotation);
        this.event = event;
    }

    public static Translation2d[] point2dToTranslation2D(CMPathPoint[] points) {
        Translation2d[] translations = new Translation2d[points.length];
        for(int i = 0; i < points.length; i++) {
            Point2D.Double point = points[i].getTranslationalPoint();
            translations[i] = new Translation2d(point.x, point.y);
        }
        return translations;
    }

    /**
     * Constructs an instance with the specified translational point, rotation, and event.
     *
     * @param translationalPoint The x and y coordnets of the path at this point
     * @param desiredRotation The desired rotation angle at this point
     */
    public CMPathPoint(Point2D.Double translationalPoint, double desiredRotation) {
        this(translationalPoint, desiredRotation, null);
    }

    public void setEvent(CMEvent event) {
        this.event = event;
    }

    /**
     * Gets the translational point.
     *
     * @return The x and y coordnets of the path at this point
     */
    public Point2D.Double getTranslationalPoint() {
        return translationalPoint;
    }

    /**
     * Gets the desired rotation. 
     *
     * @return The desired rotation angle at this point
     */
    public double getDesiredRotation() {
        return desiredRotation;
    }

    /**
     * Gets event to triggure at this point
     *
     * @return The event to triggure at this point
     */
    public CMEvent getEvent() {
        return event;
    }

    public static double wrapAngle(double angle) {
        angle = ((angle + 180) % 360 + 360) % 360;
        return angle - 180;
    }
     
}
