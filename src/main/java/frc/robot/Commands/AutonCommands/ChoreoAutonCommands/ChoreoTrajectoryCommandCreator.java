package frc.robot.Commands.AutonCommands.ChoreoAutonCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenaricConstants.RobotModeConstants;
import frc.robot.Constants.GenaricConstants.AutonConstants.ChoreoAutonConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class ChoreoTrajectoryCommandCreator {

    public static Command createChoreoTrajectoryCommand(String choreoTrajectoryFileName, DriveSubsystem driveSubsystem) {
        /*String allianceColor = RobotModeConstants.isBlueAlliance ? "" : "_red";
        ChoreoTrajectory trajectory = Choreo.getTrajectory(choreoTrajectoryFileName + allianceColor);
        return Choreo.choreoSwerveCommand(trajectory, driveSubsystem::getRobotPose,
            new PIDController(ChoreoAutonConstants.kPTranslationPIDConstant,
                ChoreoAutonConstants.kITranslationPIDConstant,
                ChoreoAutonConstants.kDTranslationPIDConstant),
            new PIDController(ChoreoAutonConstants.kPTranslationPIDConstant,
                ChoreoAutonConstants.kITranslationPIDConstant,
                ChoreoAutonConstants.kDTranslationPIDConstant),
            new PIDController(ChoreoAutonConstants.kPRotationPIDConstant,
                ChoreoAutonConstants.kIRotationPIDConstant,
                ChoreoAutonConstants.kDRotationPIDConstant),
            driveSubsystem::drive, ()->{return false;},
            driveSubsystem);
            */
        return new Command() {
            
        };
    }
}
