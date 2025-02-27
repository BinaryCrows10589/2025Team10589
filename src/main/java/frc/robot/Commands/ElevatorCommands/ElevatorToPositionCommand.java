// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.LEDUtils.LEDManager;

public class ElevatorToPositionCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final double desiredElevatorPosition;
    private final double elevatorToloranceRotations;

    public ElevatorToPositionCommand(double desiredElevatorPosition, double elevatorToloranceRotations, ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.desiredElevatorPosition = desiredElevatorPosition;
        this.elevatorToloranceRotations = elevatorToloranceRotations;
        Logger.recordOutput("Elevator/InPosition", false);
        addRequirements(this.elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.elevatorSubsystem.setDesiredElevatorPosition(desiredElevatorPosition);
        LEDManager.setSolidColor(ControlConstants.kElevatorInMotionColor);
        Logger.recordOutput("Elevator/InPosition", false);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.setDesiredElevatorPosition(desiredElevatorPosition);
        LEDManager.setSolidColor(ControlConstants.kElevatorInPositionColor);
        Logger.recordOutput("Elevator/InPosition", true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.elevatorSubsystem.isElevatorInTolorence(this.elevatorToloranceRotations);
    }

    public boolean isWithinTolerance(double tolerance) {
        return this.elevatorSubsystem.isElevatorInTolorence(tolerance);
    }
}