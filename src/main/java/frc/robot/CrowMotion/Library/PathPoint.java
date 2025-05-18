package frc.robot.CrowMotion.Library;

import java.awt.geom.Point2D;

import frc.robot.CrowMotion.CMEvent; 
public class PathPoint {

    private Point2D translationalPoint;
    private double desiredRotation;
    private CMEvent event;

    /**
     * Constructs an instance with the specified translational point, rotation, and event.
     *
     * @param translationalPoint The x and y coordnets of the path at this point
     * @param desiredRotation The desired rotation angle at this point
     * @param event The event to triggure at this point
     */
    public PathPoint(Point2D translationalPoint, double desiredRotation, CMEvent event) {
        this.translationalPoint = translationalPoint;
        this.desiredRotation = wrapAngle(desiredRotation);
        this.event = event;
    }

    /**remainder = angle % 360
`if remainder > 180:
remainder -= 360
elif remainder <= -180:
remainder += 360`
     * Constructs an instance with the specified translational point, rotation, and event.
     *
     * @param translationalPoint The x and y coordnets of the path at this point
     * @param desiredRotation The desired rotation angle at this point
     */
    public PathPoint(Point2D translationalPoint, double desiredRotation) {
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
    public Point2D getTranslationalPoint() {
        return translationalPoint;
    }

    /**
     * Gets the desired rotation. TODO: Normile this to 180 -180
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
