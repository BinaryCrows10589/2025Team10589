// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.LEDUtils.LEDManager;

public class ClimberToPositionCommand extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final double desiredClimberPosition;
    private final double climberToleranceRotations;

    public ClimberToPositionCommand(double desiredClimberPosition, double climberToleranceRotations, ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.desiredClimberPosition = desiredClimberPosition;
        this.climberToleranceRotations = climberToleranceRotations;
        addRequirements(this.climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.climberSubsystem.setDesiredClimberPosition(desiredClimberPosition);
        LEDManager.setSolidColor(ControlConstants.kClimberInMotionColor);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LEDManager.setSolidColor(ControlConstants.kClimberInPositionColor);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.climberSubsystem.isClimberInTolerance(this.climberToleranceRotations);
    }

    public boolean isWithinTolerance(double tolerance) {
        return this.climberSubsystem.isClimberInTolerance(tolerance);
    }
}