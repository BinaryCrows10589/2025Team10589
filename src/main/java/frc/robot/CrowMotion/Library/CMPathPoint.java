package frc.robot.CrowMotion.Library;

import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CrowMotion.UserSide.CMEvent;
import frc.robot.CrowMotion.UserSide.CMRotation;

public class CMPathPoint {

    private Point2D.Double translationalPoint;
    private CMRotation desiredRotation;
    private CMEvent event;
    private double distenceFromStart;

    /**
     * Constructs an instance with the specified translational point, rotation, and
     * event.
     *
     * @param translationalPoint The x and y coordnets of the path at this point
     * @param desiredRotation    The desired rotation angle at this point
     * @param event              The event to triggure at this point
     */
    public CMPathPoint(Point2D.Double translationalPoint, CMRotation desiredRotation, CMEvent event, double distenceFromStart) {
        this.translationalPoint = translationalPoint;
        this.desiredRotation = desiredRotation;
        this.event = event;
        this.distenceFromStart = distenceFromStart;
    }

    public static Translation2d[] point2dToTranslation2D(CMPathPoint[] points) {
        Translation2d[] translations = new Translation2d[points.length];
        for (int i = 0; i < points.length; i++) {
            Point2D.Double point = points[i].getTranslationalPoint();
            translations[i] = new Translation2d(point.x, point.y);
        }
        return translations;
    }

    /**
     * Constructs an instance with the specified translational point, rotation
     *
     * @param translationalPoint The x and y coordnets of the path at this point
     * @param desiredRotation The desired rotation angle at this point
     * @param distenceFromStart The distence that this point is from the start of the path
     */
    public CMPathPoint(Point2D.Double translationalPoint, double distenceFromStart) {
        this(translationalPoint, null, null, distenceFromStart);
    }


    /**
     * Gets the translational point.
     *
     * @return The x and y coordnets of the path at this point
     */
    public Point2D.Double getTranslationalPoint() {
        return translationalPoint;
    }

   
    public void setDesiredRotation(CMRotation desiredRotation) {
        this.desiredRotation = desiredRotation;
    }

    /**
     * Gets the desired rotation.
     *
     * @return The desired rotation angle at this point
     */
    public CMRotation getDesiredRotation() {
        return desiredRotation;
    }

    public void setEvent(CMEvent event) {
        this.event = event;
    }

    /**
     * Gets event to triggure at this point
     *
     * @return The event to triggure at this point
     */
    public CMEvent getEvent() {
        return event;
    }

    public double getDistenceFromStart() {
        return this.distenceFromStart;
    }

}
