// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class LockSwerves extends Command {
  
    private DriveSubsystem driveSubsystem;
  
    public LockSwerves(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(this.driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.driveSubsystem.lockSwerves();
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