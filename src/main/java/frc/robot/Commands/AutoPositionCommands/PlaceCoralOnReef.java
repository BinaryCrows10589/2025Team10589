package frc.robot.Commands.AutoPositionCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.OuttakeWheelsCommands.OuttakeCoralCommand;
import frc.robot.Deprecated.WPILibTrajectoryCommandCreator;
import frc.robot.Subsystems.Outtake.OuttakeCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;


public class PlaceCoralOnReef extends Command{
    private final OuttakeCommandFactory outtakeCommandFactory;
    private final OuttakeCoralCommand outtakeCoralCommand;

    /*
     * Goal impliment logic only once
     * be able to set fully difforent pid values for every single auto position point
     * 
     */

    public PlaceCoralOnReef(AutonPoint goalScorePosition, double maxTime, OuttakeCommandFactory outtakeCommandFactory, WPILibTrajectoryCommandCreator goToScorePosition) {
        this.outtakeCommandFactory = outtakeCommandFactory;
        this.outtakeCoralCommand = this.outtakeCommandFactory.createOuttakeCoralCommand();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
