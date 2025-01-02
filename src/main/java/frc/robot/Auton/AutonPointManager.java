package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utils.AutonUtils.WPILIBTrajectoryConfig;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class AutonPointManager {
    // Robot start positions
    public final AutonPoint kExampleStartPoint;
    public final AutonPoint kDriveToGameObjectTestAutonStartPoint;

    // Decloration of all auton points. 
    public final AutonPoint kExampleAutonPoint;
    public final AutonPoint kExampleAutonPoint1;
    public final AutonPoint kExampleAutonPoint2;
    public final AutonPoint kExampleAutonPoint3;
    public final AutonPoint kExampleAutonPoint4;
    public final AutonPoint[] kRealRobotTestAutonPointArray1 = new AutonPoint[2];
    public final AutonPoint[] kExampleAutonPointArray2 = new AutonPoint[3];

    // Decloration of PathPlanner PathNames
    public final AutonPoint kExampleAutonEndPoint = new AutonPoint(4.7, 4, 0);

    // Decloration of all WPILIB Trajectory names and configs
    public WPILIBTrajectoryConfig kExampleWpilibTrajectoryConfig;

    public AutonPointManager() {
        this.kExampleStartPoint = new AutonPoint(0, 0, 0);
        this.kDriveToGameObjectTestAutonStartPoint = new AutonPoint(2.1, 6.4, 0);

        this.kExampleAutonPoint1 = new AutonPoint(0, 0, 0);
        this.kExampleAutonPoint2 = new AutonPoint(3, 7, 0);
        this.kExampleAutonPoint3 = new AutonPoint(4.5, 7.5, 0);
        this.kExampleAutonPoint4 = new AutonPoint(4.5, 7.5 , 0);
        this.kExampleAutonPoint = new AutonPoint(3, 3, 0);


        configExampleAutonPointArray();
        configExampleWpilibTrajectoryConfig();
    }

    private void configExampleAutonPointArray() {
        this.kRealRobotTestAutonPointArray1[0] = new AutonPoint(0, 0, 0);
        this.kRealRobotTestAutonPointArray1[1] = new AutonPoint(2, 2, 0);
        this.kExampleAutonPointArray2[0] = new AutonPoint(0, 0, 0);
        this.kExampleAutonPointArray2[1] = new AutonPoint(4.876, 3.951, 0);
        this.kExampleAutonPointArray2[2] = new AutonPoint(8, 7.5, 30);
        //this.kExampleAutonPointArray2[2] = new AutonPoint(5, 8, 0);
        //this.kExampleAutonPointArray2[3] = new AutonPoint(3, 7.2, 0);
        //this.kExampleAutonPointArray[3] = this.kExampleAutonPoint4;
    } 

    private void configExampleWpilibTrajectoryConfig() {
        this.kExampleWpilibTrajectoryConfig = new WPILIBTrajectoryConfig(new double[]{1, 0, 0},
        new double[]{1, 0, 0},
        4.22, 1.2, 1, 1);
    }
}
