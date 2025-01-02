// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesChangableValue;
import frc.robot.Utils.LEDUtils.LEDManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private NetworkTablesChangableValue autonDebugMode = new NetworkTablesChangableValue("RobotMode/AutonDebugMode", RobotModeConstants.kAutonDebugMode);

        /**
         * This function is run when the robot is first started up and should be used for any
         * initialization code.
         */
        @Override
        public void robotInit() {
           // DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

            // BuiltConstatns is generated when the project is built. No imports or anything is needed if it is red underlined just build the project
            // Record metadata
            Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
            Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
            Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
            Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

            switch (BuildConstants.DIRTY) {
                case 0:
                    Logger.recordMetadata("GitDirty", "All changes committed");
                    break;
                case 1:
                    Logger.recordMetadata("GitDirty", "Uncomitted changes");
                    break;
                default:
                    Logger.recordMetadata("GitDirty", "Unknown");
                    break;
            }

            // Set up data receivers & replay source
            switch (RobotModeConstants.currentMode) {
                case REAL:
                    // Running on a real robot, log to a USB stick ("/U/logs")
                    Logger.addDataReceiver(new WPILOGWriter());
                    Logger.addDataReceiver(new NT4Publisher());
                    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
                    break;
                case SIM:
                    // Running a physics simulator, log to NT
                    Logger.addDataReceiver(new NT4Publisher());
                    break;
                case REPLAY:
                    // Replaying a log, set up replay source
                    setUseTiming(true); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog();
                    Logger.setReplaySource(new WPILOGReader(logPath));
                    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                    break;
            }

            // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
            Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

            // Initalized the LEDs
            LEDManager.init(); 

            // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
            // autonomous chooser on the dashboard.
            robotContainer = new RobotContainer();
            checkDriverStationUpdate();
            FollowPathCommand.warmupCommand().schedule();
        }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test. 
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        RobotModeConstants.kAutonDebugMode = (boolean)this.autonDebugMode.getChangableValueOnNetworkTables();
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        checkDriverStationUpdate();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {    
        if(RobotModeConstants.kAutonDebugMode) {
            this.robotContainer = new RobotContainer();
        }
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        checkDriverStationUpdate();
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
        autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {
        checkDriverStationUpdate();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
        autonomousCommand.cancel();
        }
  
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    private void checkDriverStationUpdate() {
        Optional<Alliance> currentAllianceFromDriverStation = DriverStation.getAlliance();
        Alliance currentAlliance = currentAllianceFromDriverStation.orElse(Alliance.Blue);
        // If we have data, and have a new alliance from last time
        
        if (DriverStation.isDSAttached() && currentAlliance != FieldConstants.alliance) {
            FieldConstants.alliance = currentAlliance;
            RobotModeConstants.isBlueAlliance = currentAlliance == Alliance.Blue;
            RobotModeConstants.hasAllianceChanged = true;
            this.robotContainer.updateAlliance();
        }
    }
}
