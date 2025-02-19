// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.Wait;

public class LockSwervesAuton extends Command {
  
    private DriveSubsystem driveSubsystem;
    private Wait lockTimer;
  
    public LockSwervesAuton(DriveSubsystem driveSubsystem, double lockTime) {
        this.driveSubsystem = driveSubsystem;
        this.lockTimer = new Wait(lockTime);
        addRequirements(this.driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  
        this.lockTimer.startTimer();
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
        this.lockTimer.disableTimer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.lockTimer.hasTimePassed();
    }
}