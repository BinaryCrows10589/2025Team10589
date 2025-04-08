// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class DriveForwardCommand extends Command {
  
    private DriveSubsystem driveSubsystem;
    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rotSpeed = 0;

    public DriveForwardCommand(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed, double rotSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        addRequirements(this.driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.driveSubsystem.drive(xSpeed, ySpeed, rotSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}