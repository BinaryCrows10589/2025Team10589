package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class AutonPointManager {
    // Robot start positions
    public AutonPoint kAllianceBargeStartPosition;

    // Decloration of PathPlanner PathNames
   

    // Decloration of all WPILIB Trajectory names and configs
    public WPILIBTrajectoryConfig kExampleWpilibTrajectoryConfig;

    public AutonPointManager() {
        configureStartPoints();

    }

    public void configureStartPoints() {
        this.kAllianceBargeStartPosition = new AutonPoint(7.99, 5.22, 0);
    }


}
