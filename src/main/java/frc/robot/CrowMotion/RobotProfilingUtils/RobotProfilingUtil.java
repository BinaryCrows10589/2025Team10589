package frc.robot.CrowMotion.RobotProfilingUtils;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CrowMotion.CrowMotionConfig;
import frc.robot.Utils.SwerveDriveUtils.DesiredMetersPerSecondToVoltage;

public class RobotProfilingUtil {
    
    
    public class ProfileMaxPossibleTranslationalVelocityMPSMaxPossibleAverageSwerveModuleMPS {
        /*
        * 0. Before running this profiler, ensure that your wheel circumference is configured 
            accuretly otherwise the profile will not be accurette. Furthermore, your CrowMotionConfig
            must have been configured(except for your RobotProfile of cource)
        * 1. Call this method in the TeleopPeriod method of of the Robot.java file
        * 2. Ensure that all defualt commands partaining to the swerve subsystem
        *  are disabled temporarly otherwise this will not work as the defualt command 
        *  will hold control of the swerve subsystem, preventing this from working.
        * 3. Place the robot on your practice field carpet. Ensure that the robot has 
        *    plenty of room to fully accelerate and hold its max speed. 
        *    The robot will drive forward untill it holds its max speed for 13 frames(50 frames per second)
        *    second starting on robot enable
        * WARNING, The robot wil accelerate fast, be carfull to ensure it will not tip over. 
        * If needed this method lets you limit your acceleration but that will result in more room being needed. 
        * 5. This function will log the max translational velocity to Smartdashboard.
            You will need to input this value to your RobotProfile intence that you provide to your CrowMotionConfig   
        * 6. The robot will start to decelerate slowly after the profile, just disable the robot whenver you are confinate it will not tip over
        */
        private static double desiredVelocity; 
        private static double lastStartTime = 0;
        private static double frameTime;
        private static double framesAtCurrentVelocity;
        private static double lastVelocity = 0;
        private static boolean finished = false;
        private static double startTime = 0;
        public static void profileMaxPossibleTranslationalVelocityMPSAndMaxPossibleAverageSwerveModuleMPS(double limitedAcceleration) {
            if(startTime == 0) {
                startTime = System.currentTimeMillis();
            }
            if(lastStartTime == 0) {
                lastStartTime = System.currentTimeMillis();
            }
            frameTime = (System.currentTimeMillis() - lastStartTime) / 1000.0;
            lastStartTime = System.currentTimeMillis();
            if(limitedAcceleration == -1 && !finished) {
                desiredVelocity = 10;
            } else if (!finished) {
                desiredVelocity += limitedAcceleration * frameTime;
                if(desiredVelocity > 10) {
                    desiredVelocity = 10;
                }
            } else {
                if(desiredVelocity > 0) {
                    desiredVelocity -= desiredVelocity * frameTime;
                    desiredVelocity = 0;
                }
            }
            CrowMotionConfig.setRobotVelocityMPSandDPS(desiredVelocity, 0, 0);
            double currentVelocity = Math.sqrt(Math.pow(CrowMotionConfig.getRobotVelocityMPSandDPS()[0], 2) + 
                Math.pow(CrowMotionConfig.getRobotVelocityMPSandDPS()[1], 2));
            boolean inTolorence = Math.abs(currentVelocity - lastVelocity) < .005;
            lastVelocity = currentVelocity;
            
            if(inTolorence && currentVelocity != 0) {
                framesAtCurrentVelocity++;
            } else {
                framesAtCurrentVelocity = 0;
            }

            if(framesAtCurrentVelocity >= 13 && !finished) {
                SmartDashboard.putNumber("CrowMotion/RobotProfile/MaxPossibleTranslationalVelocityMPS", currentVelocity);
                SmartDashboard.putNumber("CrowMotion/RobotProfile/MaxPossibleAverageSwerveModuleMPS", CrowMotionConfig.getAverageSwerveModuleVelocityMPS());
                finished = true;
            }
        }

        public static void profileMaxPossibleTranslationalVelocityMPS() {
            profileMaxPossibleTranslationalVelocityMPSAndMaxPossibleAverageSwerveModuleMPS(-1);
        }
    }
    
}
